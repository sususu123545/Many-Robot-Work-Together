#!/usr/bin/env python3
"""Serve the latest compressed camera frame over HTTP, on the car.

WHY THIS EXISTS
    web_video_server (ROS 2) leaks/stalls its MJPEG handler whenever a client
    disconnects mid-write: after a few abrupt disconnects the node spins at
    high CPU and stops answering, which is what made the console camera go
    black while /scan kept working. This relay subscribes to the camera's
    already-encoded JPEG topic (image_transport /compressed) and serves the
    latest frame with a dead-simple, thread-per-request HTTP server that has
    per-write error handling and no shared encoder lock. It only reads the
    ROS topic; it never commands the robot.

ENDPOINTS
    GET /frame    -> newest JPEG (503 when no fresh frame)
    GET /stream   -> multipart MJPEG fan-out (compatibility)
    GET /health   -> {"seq":N,"age":seconds,"topic":...}
"""
import http.server
import json
import os
import socketserver
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

PORT = int(os.environ.get('FRAME_RELAY_PORT', '8099'))
TOPIC = os.environ.get('FRAME_RELAY_TOPIC', '/rear_camera/image_raw/compressed')
STALE_SECONDS = 5.0

LOCK = threading.Lock()
STATE = {'frame': None, 'time': 0.0, 'seq': 0}


class Listener(Node):
    def __init__(self):
        super().__init__('frame_relay')
        self.create_subscription(CompressedImage, TOPIC, self.on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info('frame relay listening on %s' % TOPIC)

    def on_image(self, message):
        frame = bytes(message.data)
        if not frame:
            return
        with LOCK:
            STATE['frame'] = frame
            STATE['time'] = time.monotonic()
            STATE['seq'] += 1


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *args):
        pass

    def _latest(self):
        with LOCK:
            return STATE['frame'], STATE['time'], STATE['seq']

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path in ('/', '/frame', '/snapshot', '/index.html'):
            frame, sampled, _seq = self._latest()
            if frame is None or (time.monotonic() - sampled) > STALE_SECONDS:
                self.send_response(503)
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(frame)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(frame)
            except OSError:
                pass
            return
        if path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()
            last = -1
            try:
                while True:
                    frame, _sampled, seq = self._latest()
                    if frame is None:
                        time.sleep(0.03)
                        continue
                    if seq == last:
                        time.sleep(0.01)
                        continue
                    last = seq
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(frame)).encode()
                                     + b'\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
            except OSError:
                pass
            finally:
                self.close_connection = True
            return
        if path == '/health':
            with LOCK:
                seq, sampled = STATE['seq'], STATE['time']
            body = json.dumps({
                'seq': seq,
                'age': round(time.monotonic() - sampled, 3) if sampled else None,
                'topic': TOPIC,
            }).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)


def main():
    rclpy.init()
    node = Listener()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    socketserver.ThreadingMixIn.daemon_threads = True
    server = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print('frame relay listening on 0.0.0.0:%d topic=%s' % (PORT, TOPIC), flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
