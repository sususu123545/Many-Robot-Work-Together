"""Offline tests for actual request payloads, asynchronous jobs and boundaries."""
import base64
from email.message import Message
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web_console'))
from vision_bridge import VisionBridge, VisionError, validate_observation, MAX_IMAGE_BYTES


class Reply(io.BytesIO):
    def __enter__(self): return self
    def __exit__(self, *args): self.close()


class ModelServer:
    def __init__(self, text='红色小球在画面下方偏左。', gate=None):
        self.text = text
        self.requests = []
        self.gate = gate

    def open(self, request, timeout):
        url = request.full_url
        if url.endswith('/models'):
            return Reply(json.dumps({'data': [{'id': 'test-vision-model'}]}).encode())
        if url.endswith('/props'):
            return Reply(b'{"modalities":{"vision":true}}')
        self.requests.append(json.loads(request.data))
        if self.gate:
            self.gate.wait(3)
        return Reply(json.dumps({'choices': [{'message': {'content': self.text}, 'finish_reason': 'stop'}],
                                 'usage': {'completion_tokens': 15}}).encode())


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config = Path(self.directory.name) / 'config.json'
        self.config.write_text(json.dumps({'base_url': 'http://127.0.0.1:8080/v1', 'name': 'test'}))
        self.payload = {'instruction': '寻找红球', 'captured_at': time.time(),
                        'image_base64': base64.b64encode(b'\xff\xd8test\xff\xd9').decode()}

    def finished(self, bridge, job_id):
        end = time.monotonic() + 3
        while time.monotonic() < end:
            job = bridge.job(job_id)
            if job['status'] != 'running':
                return job
            time.sleep(.01)
        self.fail('Job did not finish')

    def test_image_reaches_model_without_action_tools(self):
        server = ModelServer()
        bridge = VisionBridge(self.config, server)
        job = self.finished(bridge, bridge.submit(self.payload))
        self.assertEqual(job['status'], 'succeeded')
        self.assertEqual(job['result']['text'], server.text)
        sent = server.requests[0]
        self.assertNotIn('tools', sent)
        self.assertEqual(sent['messages'][1]['content'][1]['image_url']['url'],
                         'data:image/jpeg;base64,' + self.payload['image_base64'])
        self.assertEqual(sent['model'], 'test-vision-model')
        self.assertEqual(job['captured_at'], self.payload['captured_at'])
        self.assertEqual(len(job['image_sha256']), 64)
        self.assertNotIn('image_base64', job)

    def test_concurrent_request_is_rejected_with_existing_job(self):
        gate = threading.Event()
        bridge = VisionBridge(self.config, ModelServer(gate=gate))
        first = bridge.submit(self.payload)
        try:
            with self.assertRaises(VisionError) as caught:
                bridge.submit(self.payload)
            self.assertEqual(caught.exception.status, 409)
            self.assertEqual(caught.exception.job_id, first)
        finally:
            gate.set()
            self.finished(bridge, first)

    def test_old_or_invalid_camera_data_is_rejected(self):
        for changes in ({'captured_at': time.time()-20}, {'captured_at': float('nan')},
                        {'captured_at': True}, {'image_base64': 'not-base64!'},
                        {'image_base64': base64.b64encode(b'not a JPEG').decode()},
                        {'image_base64': 'A' * (MAX_IMAGE_BYTES * 2)},
                        {'instruction': ''}, {'instruction': 'a'*1001}):
            with self.subTest(changes=list(changes)):
                with self.assertRaises(VisionError):
                    validate_observation({**self.payload, **changes})

    def test_model_failure_and_empty_answer_release_worker(self):
        server = ModelServer(text='')
        bridge = VisionBridge(self.config, server)
        job = self.finished(bridge, bridge.submit(self.payload))
        self.assertEqual(job['status'], 'failed')
        end = time.monotonic() + 1
        while bridge.active_job_id and time.monotonic() < end:
            time.sleep(.01)
        server.text = '现在能看到红球。'
        self.assertEqual(self.finished(bridge, bridge.submit(self.payload))['status'], 'succeeded')

    def test_nonvision_model_is_not_called(self):
        server = ModelServer()
        bridge = VisionBridge(self.config, server)
        bridge.info_cache = {'available': False, 'message': '图片输入未启用'}
        bridge.info_at = time.monotonic()
        job = self.finished(bridge, bridge.submit(self.payload))
        self.assertEqual(job['status'], 'failed')
        self.assertEqual(server.requests, [])

    def test_post_requires_same_origin_and_bounded_body(self):
        spec = importlib.util.spec_from_file_location('sensor_proxy', ROOT/'web_console'/'proxy_server.py')
        proxy = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(proxy)
        class Handler:
            client_address = ('127.0.0.1', 12345)
            headers = Message()
            close_connection = False
        handler = Handler()
        handler.headers['Host'] = '127.0.0.1:8000'
        handler.headers['Origin'] = 'http://unrelated.example'
        with patch.object(proxy, '_send_json') as send:
            proxy._analyze_camera(handler)
            self.assertEqual(send.call_args.args[1], 403)
        handler.headers.replace_header('Origin', 'http://127.0.0.1:8000')
        handler.headers['Content-Length'] = str(10*1024*1024)
        with patch.object(proxy, '_send_json') as send:
            proxy._analyze_camera(handler)
            self.assertEqual(send.call_args.args[1], 413)


if __name__ == '__main__':
    unittest.main()
