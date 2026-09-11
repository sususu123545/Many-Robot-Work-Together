#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""从 ArmPi Pro 的 web_video_server (:8080) 抓一张真实画面。

用法:
    python car_grab.py --ip 10.120.150.178 \
        --out "D:\\多车协作\\.workbuddy\\shots\\cam.jpg"
    python car_grab.py --ip <IP> --topic /ascamera_hp60c/rgb0/image --frames 8

为什么要用流而不是 /snapshot
--------------------------------------------------------------------------
web_video_server 的 /snapshot 每次都是一次**新订阅**，而安思疆 SDK 是
「按需推流」（无订阅者就 stopStreaming），于是每次快照几乎必然命中**全黑的第 0 帧**
（gain 要在首帧回调之后才生效）。/snapshot 实测约 5~7 KB，全黑。
/stream 则能拿到连续真帧。**用文件大小就能判别**：黑帧 ~7KB，真图 20~35KB。

所以本脚本始终走 /stream，丢弃小于 --min-bytes 的帧。

强制直连: 环境里若有 http_proxy（VPN/抓包/公司代理），urllib 会把请求
打到代理上导致 502 / WinError 10061，因此这里显式禁用代理。
"""
import argparse
import os
import sys
import urllib.parse
import urllib.request

DIRECT = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def split_jpegs(buf):
    """从 MJPEG 字节流里切出完整 JPEG（FFD8 ... FFD9）。返回 (frames, rest)。"""
    frames = []
    while True:
        a = buf.find(b"\xff\xd8")
        if a < 0:
            # 丢掉 FFD8 之前的垃圾，但保留末字节以防跨块边界
            buf = buf[-1:] if buf else b""
            break
        b = buf.find(b"\xff\xd9", a + 2)
        if b < 0:
            buf = buf[a:]          # 不完整的一帧，留着下轮再拼
            break
        frames.append(buf[a:b + 2])
        buf = buf[b + 2:]
    return frames, buf


def grab(ip, topic, want_frames, min_bytes, timeout):
    url = "http://%s:8080/stream?topic=%s&type=mjpeg" % (
        ip, urllib.parse.quote(topic))
    req = urllib.request.Request(url, headers={"User-Agent": "car_grab/1.0"})
    up = DIRECT.open(req, timeout=timeout)

    frames, buf, reads = [], b"", 0
    try:
        while len(frames) < want_frames and reads < 600:
            chunk = up.read(8192)
            if not chunk:
                break
            reads += 1
            buf += chunk
            got, buf = split_jpegs(buf)
            frames.extend(got)
    finally:
        try:
            up.close()
        except Exception:
            pass
    return url, frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--topic", default="/ascamera_hp60c/rgb0/image")
    ap.add_argument("--out", default="", help="不指定则只打印统计")
    ap.add_argument("--frames", type=int, default=6)
    ap.add_argument("--min-bytes", type=int, default=15000,
                    help="小于此字节数的帧视为黑帧，直接丢弃")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()

    try:
        url, frames = grab(args.ip, args.topic, args.frames,
                           args.min_bytes, args.timeout)
    except Exception as e:  # noqa: BLE001
        print("抓流失败: %s: %s" % (type(e).__name__, e))
        print("  排查: 车上 8080 通吗？话题在 web_video_server 索引里吗？"
              "节点是不是没在跑（相机节点是手动启动的，重启就丢）？")
        return 1

    print("URL: %s" % url)
    print("收到 %d 帧:" % len(frames))
    for i, f in enumerate(frames):
        flag = "" if len(f) >= args.min_bytes else "   <- 疑似黑帧，丢弃"
        print("  frame%d  %d bytes%s" % (i, len(f), flag))

    good = [f for f in frames if len(f) >= args.min_bytes]
    if not good:
        print("!! 全部帧都偏小 —— 大概率又是全黑的第 0 帧。")
        print("   若车上有其他图像话题（如 /visual_processing/image_result）"
              "可先换话题验证链路。")
        return 2

    if args.out:
        d = os.path.dirname(os.path.abspath(args.out))
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.out, "wb") as fh:
            fh.write(good[0])
        print("已保存: %s (%d bytes)" % (args.out, len(good[0])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
