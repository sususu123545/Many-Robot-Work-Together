#!/usr/bin/env python3
# 本地代理服务：页面从 127.0.0.1 取视频/截图，由本服务直连小车转发
# 绕过 Edge 系统代理对内网 HTTP 的拦截（localhost 默认不走代理）
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import socketserver

PAGE = 'robot_console.html'
CAR_TIMEOUT = 8

# 强制直连小车，忽略任何系统/环境代理设置。
# 本脚本的目的就是绕开代理，所以它自己访问小车时也必须绕开：
# 否则环境里一旦存在 http_proxy（系统代理、VPN、抓包工具等），
# urllib 会把 http://<小车IP>:8080 的请求打到代理上，结果就是
# 502 Bad Gateway 或 WinError 10061（目标计算机积极拒绝）。
DIRECT = urllib.request.build_opener(urllib.request.ProxyHandler({}))

class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def _car_url(self, path):
        from urllib.parse import urlparse, parse_qs
        q = parse_qs(urlparse(self.path).query)
        ip = q.get('ip', [''])[0]
        topic = q.get('topic', ['/usb_cam/image_raw'])[0]
        return 'http://%s:8080%s?topic=%s&type=mjpeg' % (ip, path, urllib.parse.quote(topic))

    def do_GET(self):
        from urllib.parse import urlparse
        path = urlparse(self.path).path
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

    def log_message(self, fmt, *args):
        pass  # 静默访问日志

if __name__ == '__main__':
    socketserver.ThreadingMixIn.daemon_threads = True
    srv = ThreadingHTTPServer(('127.0.0.1', 8000), Handler)
    print('proxy ready: http://127.0.0.1:8000')
    srv.serve_forever()
