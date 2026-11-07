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
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from core_profile import load_core_profile
from vision_bridge import VisionBridge, VisionError, MAX_REQUEST_BYTES

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
VISION = VisionBridge()
FRAME_HUB = None


class FrameHub:
    """Hold ONE stable MJPEG upstream to the car and cache the latest JPEG.

    WHY: web_video_server leaks a server-side handler whenever a client
    disconnects in the middle of an MJPEG stream. The browser used to connect
    directly, so every page reload / stalled-stream reconnect leaked one
    handler; after 20+ leaks the node spun at 65% CPU and the camera went
    black while /scan (rosbridge) kept working. With this hub the car only
    ever sees a single long-lived client, and clients read the cached frame
    from /frame or join the fan-out at /stream.
    """

    def __init__(self, topic='/rear_camera/image_raw'):
        self.topic = topic
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.sequence = 0
        self.connected = False
        self.last_error = ''
        self.thread = threading.Thread(target=self._run, name='frame-hub',
                                       daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.frame, self.frame_time, self.sequence

    def info(self):
        with self.lock:
            return {'connected': self.connected,
                    'frames': self.sequence,
                    'age': round(time.monotonic() - self.frame_time, 3)
                    if self.frame_time else None,
                    'last_error': self.last_error}

    def _car_ip(self):
        try:
            status = json.loads(STATUS_FILE.read_text(encoding='utf-8'))
            ip = status.get('ip')
            if isinstance(ip, str) and ip.count('.') == 3:
                return ip
        except (OSError, ValueError):
            pass
        return ''

    def _run(self):
        """Read the car's frame-relay stream; fall back to a one-shot 8080 JPEG.

        WHY NOT THE CAR'S MJPEG DIRECTLY: web_video_server stalls permanently
        when a client disconnects mid-stream. The relay is our own tiny
        subscriber that serves the camera's already-encoded JPEG and tolerates
        client aborts, so it is safe to keep one long-lived stream from it.
        """
        import urllib.parse
        while True:
            ip = self._car_ip()
            if not ip:
                time.sleep(2.0)
                continue
            response = None
            try:
                url = 'http://%s:8099/stream' % ip
                request = urllib.request.Request(
                    url, headers={'User-Agent': 'armpi-frame-hub/1.0'})
                response = DIRECT.open(request, timeout=5)
                buffer = b''
                last_data = time.monotonic()
                while True:
                    chunk = response.read(8192)
                    if not chunk:
                        raise OSError('relay stream closed')
                    buffer += chunk
                    if len(buffer) > 4 * 1024 * 1024:
                        buffer = buffer[-1024 * 1024:]
                    while True:
                        start = buffer.find(b'\xff\xd8')
                        if start < 0:
                            break
                        end = buffer.find(b'\xff\xd9', start + 2)
                        if end < 0:
                            if start:
                                buffer = buffer[start:]
                            break
                        frame = buffer[start:end + 2]
                        buffer = buffer[end + 2:]
                        with self.lock:
                            self.frame = frame
                            self.frame_time = time.monotonic()
                            self.sequence += 1
                    now = time.monotonic()
                    if now - last_data > 10.0:
                        raise OSError('relay stream stalled')
                    last_data = now
                    self.connected = True
                    self.last_error = ''
            except Exception as error:                       # noqa: BLE001
                self.connected = False
                self.last_error = 'relay stream: %s: %s' % (type(error).__name__, error)
                fallback = ('http://%s:8080/snapshot?topic=%s&type=jpeg&qos_profile=sensor_data'
                            % (ip, urllib.parse.quote(self.topic)))
                try:
                    request = urllib.request.Request(
                        fallback, headers={'User-Agent': 'armpi-frame-hub/1.0'})
                    response = DIRECT.open(request, timeout=CAR_TIMEOUT)
                    try:
                        frame = response.read()
                    finally:
                        response.close()
                    if frame[:2] == b'\xff\xd8':
                        with self.lock:
                            self.frame = frame
                            self.frame_time = time.monotonic()
                            self.sequence += 1
                        self.last_error += '; 8080 snapshot fallback ok'
                except Exception as exc2:                    # noqa: BLE001
                    self.last_error += '; 8080 fallback: %s: %s' % (type(exc2).__name__, exc2)
                finally:
                    if response is not None:
                        try:
                            response.close()
                        except Exception:                    # noqa: BLE001
                            pass
                time.sleep(1.0)



def _local_model_request(handler):
    try:
        local = ipaddress.ip_address(handler.client_address[0]).is_loopback
    except ValueError:
        return False
    host = handler.headers.get('Host', '')
    return (local and host in ('127.0.0.1:8000', 'localhost:8000')
            and handler.headers.get('Origin', '') == 'http://' + host)


def _analyze_camera(handler):
    if not _local_model_request(handler):
        handler.close_connection = True
        _send_json(handler, 403, {'ok': False, 'message': '仅允许本机控制台提交相机画面'})
        return
    try:
        length = int(handler.headers.get('Content-Length', '0'))
        if not 0 < length <= MAX_REQUEST_BYTES:
            raise VisionError('图像请求为空或过大', 413)
        if handler.headers.get_content_type() != 'application/json':
            raise VisionError('需要 application/json 请求')
        handler.connection.settimeout(10)
        payload = json.loads(handler.rfile.read(length))
        job_id = VISION.submit(payload)
        _send_json(handler, 202, {'ok': True, 'job_id': job_id})
    except VisionError as error:
        handler.close_connection = True
        _send_json(handler, error.status, {'ok': False, 'message': str(error), 'job_id': error.job_id})
    except (ValueError, UnicodeError, OSError):
        handler.close_connection = True
        _send_json(handler, 400, {'ok': False, 'message': '请求格式无效或读取超时'})


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
        qos = '&qos_profile=sensor_data' if load_core_profile()['ros_version'] == 2 else ''
        image_type = 'jpeg' if path == '/snapshot' else 'mjpeg'
        return 'http://%s:8080%s?topic=%s&type=%s%s' % (ip, path, urllib.parse.quote(topic), image_type, qos)

    def do_GET(self):
        from urllib.parse import urlparse
        path = urlparse(self.path).path
        if path == '/api/model':
            try:
                _send_json(self, 200, VISION.info())
            except (VisionError, ValueError, OSError) as error:
                _send_json(self, 503, {'available': False, 'message': str(error)})
            return
        if path.startswith('/api/vision/jobs/'):
            try:
                _send_json(self, 200, VISION.job(path.rsplit('/', 1)[-1]))
            except VisionError as error:
                _send_json(self, error.status, {'ok': False, 'message': str(error)})
            return
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

        if path in ('/', '/robot_console.html', '/ros2_console.html', '/sensor_console.html'):
            try:
                profile = load_core_profile()
                page = 'sensor_console.html' if path in ('/sensor_console.html', '/ros2_console.html') or profile['ros_version'] == 2 else PAGE
                body = open(page, 'rb').read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except Exception as e:
                self.send_error(500, str(e))
            return

        if path == '/api/video':
            _send_json(self, 200, FRAME_HUB.info())
            return

        # 最新帧：页面高频轮询（~12 fps），每次都是一个完整、短小的请求。
        if path == '/frame':
            frame, frame_time, _sequence = FRAME_HUB.snapshot()
            if frame is None or (time.monotonic() - frame_time) > 5.0:
                _send_json(self, 503, {'ok': False, 'message': '相机画面尚未就绪',
                                       'hub': FRAME_HUB.info()})
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(frame)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(frame)
            return

        # 兼容旧 <img src=/stream>：从帧总线 fan-out，绝不再直连小车。
        if path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()
            last = -1
            try:
                while True:
                    frame, _frame_time, sequence = FRAME_HUB.snapshot()
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    if sequence == last:
                        time.sleep(0.02)
                        continue
                    last = sequence
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(frame)).encode()
                                     + b'\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
            except Exception:
                pass  # 客户端断开
            finally:
                self.close_connection = True
            return

        # 一次性快照：直连小车，短请求，读失败也保证关闭上游。
        if path == '/snapshot':
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
            try:
                body = up.read()
            except Exception as error:
                body = ('proxy error: %s' % error).encode()
                self.send_response(502)
                self.send_header('Content-Type', 'text/plain; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            finally:
                try:
                    up.close()
                except Exception:
                    pass
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_error(404)

    def do_POST(self):
        from urllib.parse import urlparse
        if urlparse(self.path).path == '/api/vision/analyze':
            _analyze_camera(self)
            return
        if urlparse(self.path).path == '/api/repair-tunnel':
            _request_tunnel_repair(self)
            return
        self.send_error(404)

    def log_message(self, fmt, *args):
        pass  # 静默访问日志

if __name__ == '__main__':
    socketserver.ThreadingMixIn.daemon_threads = True
    FRAME_HUB = FrameHub()
    srv = ThreadingHTTPServer(('127.0.0.1', 8000), Handler)
    print('proxy ready: http://127.0.0.1:8000')
    srv.serve_forever()
