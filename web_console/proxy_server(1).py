#!/usr/bin/env python3
# 本地代理服务：页面从 127.0.0.1 取视频/截图，由本服务直连小车转发
# 绕过 Edge 系统代理对内网 HTTP 的拦截（localhost 默认不走代理）
import urllib.request
import ipaddress
import json
import os
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import socketserver

PAGE = 'robot_console.html'
CAR_TIMEOUT = 8

# 强制直连小车，忽略任何系统/环境代理设置。
# 本脚本的目的就是绕开代理，所以它自己访问小车时也必须绕开：
# 否则环境里一旦存在 http_proxy（系统代理、VPN、抓包工具等），
# urllib 会把 http://<小车IP>:8080 的请求打到代理上，结果就是
# 502 Bad Gateway 或 WinError 10061（目标计算机积极拒绝）。
DIRECT = urllib.request.build_opener(urllib.request.ProxyHandler({}))
STATUS_FILE = Path(__file__).resolve().parents[1] / 'tools' / 'runtime' / 'robot_status.json'
REPAIR_REQUEST_FILE = Path(__file__).resolve().parents[1] / 'tools' / 'runtime' / 'repair_tunnel.request.json'
REPAIR_LOCK = threading.Lock()


def _send_json(handler, status, payload):
    body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
    handler.send_response(status)
    handler.send_header('Content-Type', 'application/json; charset=utf-8')
    handler.send_header('Cache-Control', 'no-store')
    handler.send_header('Content-Length', str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _request_tunnel_repair(handler):
    """Queue a loopback-only request for the supervisor to rebuild SSH tunnel."""
    try:
        peer = ipaddress.ip_address(handler.client_address[0])
    except (ValueError, IndexError):
        _send_json(handler, 403, {'ok': False, 'message': '仅允许本机请求'})
        return
    host = handler.headers.get('Host', '')
    origin = handler.headers.get('Origin', '')
    if (not peer.is_loopback or host not in ('127.0.0.1:8000', 'localhost:8000')
            or origin != 'http://' + host):
        _send_json(handler, 403, {'ok': False, 'message': '请求来源校验失败'})
        return

    try:
        status = json.loads(STATUS_FILE.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        _send_json(handler, 503, {'ok': False, 'message': '未读取到本机隧道监督状态'})
        return

    heartbeat = status.get('supervisor_heartbeat', 0)
    try:
        heartbeat = float(heartbeat)
    except (TypeError, ValueError):
        heartbeat = 0
    if time.time() - heartbeat > 15:
        _send_json(handler, 503, {'ok': False, 'message': '本机隧道监督进程未运行或状态已过期'})
        return
    if status.get('state') == 'port_conflict':
        _send_json(handler, 409, {'ok': False, 'message': '本机 9090 被其他程序占用，未触碰该程序'})
        return
    repair = status.get('repair') or {}
    if repair.get('state') in ('restarting', 'verifying'):
        _send_json(handler, 409, {'ok': False, 'message': '连接修复正在进行，请稍候'})
        return
    if not status.get('ip') or status.get('state') in ('searching', 'stopped'):
        _send_json(handler, 409, {'ok': False, 'message': '尚未确认小车 IP；请先连接热点并等待自动发现'})
        return

    with REPAIR_LOCK:
        try:
            if REPAIR_REQUEST_FILE.exists():
                pending = json.loads(REPAIR_REQUEST_FILE.read_text(encoding='utf-8'))
                age = time.time() - float(pending.get('requested_at', 0))
                if age <= 30:
                    _send_json(handler, 409, {'ok': False, 'message': '连接修复请求正在处理中，请稍候'})
                    return
        except (OSError, ValueError, TypeError):
            pass

        request_id = uuid.uuid4().hex
        payload = {
            'request_id': request_id,
            'host': status['ip'],
            'requested_at': time.time(),
        }
        try:
            REPAIR_REQUEST_FILE.parent.mkdir(parents=True, exist_ok=True)
            temporary = REPAIR_REQUEST_FILE.with_suffix('.tmp')
            temporary.write_text(json.dumps(payload), encoding='utf-8')
            os.replace(str(temporary), str(REPAIR_REQUEST_FILE))
        except OSError:
            _send_json(handler, 503, {'ok': False, 'message': '无法向本机隧道监督进程发送修复请求'})
            return

    _send_json(handler, 202, {
        'ok': True,
        'request_id': request_id,
        'message': '已请求重建本机 SSH 隧道；控制台将自动重新连接 ROS',
    })

class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def _car_url(self, path):
        from urllib.parse import urlparse, parse_qs
        q = parse_qs(urlparse(self.path).query)
        ip = q.get('ip', [''])[0]
        topic = q.get('topic', ['/rear_camera/image_raw'])[0]
        return 'http://%s:8080%s?topic=%s&type=mjpeg' % (ip, path, urllib.parse.quote(topic))

    def do_GET(self):
        from urllib.parse import urlparse
        path = urlparse(self.path).path
        if path == '/api/robot':
            try:
                status = json.loads(STATUS_FILE.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                status = {'state': 'starting', 'ip': None,
                          'message': '正在启动小车自动发现'}
            # Advertise endpoint support so a cached/old supervisor or proxy
            # cannot leave a visible button that only fails after being clicked.
            status['repair_supported'] = True
            body = json.dumps(status, ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path in ('/', '/robot_console.html'):
            try:
                body = open(PAGE, 'rb').read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except Exception as e:
                self.send_error(500, str(e))
            return

        if path in ('/stream', '/snapshot'):
            target = self._car_url(path)
            try:
                req = urllib.request.Request(target, headers={'User-Agent': 'local-proxy/1.0'})
                up = DIRECT.open(req, timeout=CAR_TIMEOUT)
            except Exception as e:
                body = ('proxy error: %s' % e).encode()
                self.send_response(502)
                self.send_header('Content-Type', 'text/plain; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == '/snapshot':
                body = up.read()
                up.close()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            # MJPEG 流：透传 multipart 内容
            self.send_response(200)
            self.send_header('Content-Type', up.headers.get('Content-Type', 'multipart/x-mixed-replace; boundary=frame'))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                while True:
                    chunk = up.read(4096)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
            except Exception:
                pass  # 客户端断开或上游断开
            finally:
                up.close()
            return

        self.send_error(404)

    def do_POST(self):
        from urllib.parse import urlparse
        if urlparse(self.path).path == '/api/repair-tunnel':
            _request_tunnel_repair(self)
            return
        self.send_error(404)

    def log_message(self, fmt, *args):
        pass  # 静默访问日志

if __name__ == '__main__':
    socketserver.ThreadingMixIn.daemon_threads = True
    srv = ThreadingHTTPServer(('127.0.0.1', 8000), Handler)
    print('proxy ready: http://127.0.0.1:8000')
    srv.serve_forever()
