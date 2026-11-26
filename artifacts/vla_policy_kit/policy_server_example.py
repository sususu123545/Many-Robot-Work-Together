"""Reference HTTP policy server for the ArmPi Pro real-robot test.

Run it in --mock mode first: it answers with a synthetic but *shape-correct*
block, so the robot side can be exercised end to end before the checkpoint is
wired in.  Then replace the three lines marked ADAPT with your own inference and
delete --mock from the command line.

    python policy_server_example.py --mock --port 8000
    python policy_server_example.py --ckpt logs/t08/runs/.../ckpt/checkpoints/003000 --port 8000

Contract: see CONTRACT.md in this kit.  Two rules that matter more than the rest:

* `output_kind` must be "physical_radians".  De-normalise here; the robot refuses
  normalised values because it cannot know your dataset's statistics.
* `control_dt` must be 0.5 s (2 Hz).  That is the rate the demonstration data was
  recorded at, and the arm bus accepts anything in 0.3..5.0 s.

No dependency outside the standard library is needed in --mock mode, so this file
runs on any Python 3.8+.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

#: Must match the joint order in the training dataset: joint1..joint5, gripper.
JOINT_ORDER = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')

#: What the demonstration data considered "arm at the ready pose".  The mock uses
#: it so the numbers look like real ones; delete once the model is wired in.
READY_POSE_RADIANS = (-0.046, 0.758, -1.780, -1.273, -0.008, -1.756)

CHUNK_STEPS = 10          # 10 steps x 0.5 s = 5 s of motion per request
CONTROL_DT = 0.5


class Policy:
    """Wraps the checkpoint.  Only this class knows about lerobot."""

    def __init__(self, checkpoint: str, mock: bool):
        self.mock = mock
        self.policy_id = 'mock' if mock else 'realkit-' + checkpoint.rstrip('/').split('/')[-1]
        self._step = 0
        self._policy = None
        if not mock:
            self._load(checkpoint)

    def _load(self, checkpoint: str) -> None:
        # ------------------------------------------------------------------
        # ADAPT 1/3 -- load the checkpoint.
        # ------------------------------------------------------------------
        import torch                                                    # noqa: F401
        from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy

        self._policy = SmolVLAPolicy.from_pretrained(checkpoint)
        self._policy.eval()
        self.device = 'cuda' if __import__('torch').cuda.is_available() else 'cpu'
        self._policy.to(self.device)

    def act(self, observation: dict) -> dict:
        if self.mock:
            return self._mock(observation)

        # ------------------------------------------------------------------
        # ADAPT 2/3 -- turn the wire observation into your training-time batch.
        # The keys must be exactly the ones the dataset used, or the policy sees
        # a different world: observation.state (6,) and
        # observation.images.camera1 (3, H, W) in [0, 1].
        # ------------------------------------------------------------------
        import base64
        import io

        import numpy as np
        import torch
        from PIL import Image

        state = torch.tensor(observation['state'], dtype=torch.float32).unsqueeze(0)
        raw = base64.b64decode(observation['image_b64'])
        image = Image.open(io.BytesIO(raw)).convert('RGB').resize((640, 480))
        pixels = torch.from_numpy(np.asarray(image, dtype=np.float32) / 255.0)
        pixels = pixels.permute(2, 0, 1).unsqueeze(0)

        batch = {
            'observation.state': state.to(self.device),
            'observation.images.camera1': pixels.to(self.device),
            'task': [observation.get('task', 'Pick up the red ball.')],
        }

        # ------------------------------------------------------------------
        # ADAPT 3/3 -- predict a chunk and hand back physical radians.
        # `predict_action_chunk` returns (1, N, 6) in the dataset's own units
        # when the normalisation is baked into the checkpoint.  Verify that
        # against one sample from the dataset before trusting it.
        # ------------------------------------------------------------------
        with torch.no_grad():
            chunk = self._policy.predict_action_chunk(batch)
        actions = chunk.squeeze(0).cpu().numpy().tolist()
        if actions and len(actions[0]) != len(JOINT_ORDER):
            raise RuntimeError('policy returned %d components, expected %d'
                               % (len(actions[0]), len(JOINT_ORDER)))

        return {
            'actions': [[float(v) for v in row] for row in actions],
            'policy_id': self.policy_id,
            'control_dt': CONTROL_DT,
            'validity_s': CONTROL_DT * len(actions) + 1.0,
            'output_kind': 'physical_radians',
        }

    def _mock(self, observation: dict) -> dict:
        """Shape-correct stand-in: a slow, small drift towards the ball pose.

        Not a policy -- it exists so the robot side can be tested (and its safety
        gate exercised) before a checkpoint is available.  The motion is small on
        purpose: if the arm lurches on mock output, the gate is not working.
        """
        self._step += 1
        base = observation.get('state') or list(READY_POSE_RADIANS)
        actions = []
        for step in range(CHUNK_STEPS):
            drift = (self._step * 0.02) + step * 0.004
            row = list(base)
            row[1] += math.sin(drift) * 0.02        # joint2: a few tick worth
            row[3] += math.sin(drift * 1.3) * 0.02  # joint4
            actions.append([round(v, 6) for v in row])
        return {
            'actions': actions,
            'policy_id': self.policy_id,
            'control_dt': CONTROL_DT,
            'validity_s': CONTROL_DT * CHUNK_STEPS + 1.0,
            'output_kind': 'physical_radians',
        }


def make_handler(policy: Policy):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def do_POST(self):                      # noqa: N802 - stdlib naming
            if self.path not in ('/act', '/'):
                self.send_error(404, 'POST /act only')
                return
            length = int(self.headers.get('Content-Length') or 0)
            body = self.rfile.read(length)
            started = time.monotonic()
            try:
                observation = json.loads(body.decode('utf-8'))
                if 'state' not in observation or 'image_b64' not in observation:
                    raise ValueError('observation needs both state and image_b64')
                answer = policy.act(observation)
            except Exception as error:          # noqa: BLE001 - report to client
                payload = json.dumps({'error': str(error)}).encode('utf-8')
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                print('  500 %s' % error, flush=True)
                return

            payload = json.dumps(answer).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            print('  %d actions  %.0f ms'
                  % (len(answer['actions']), (time.monotonic() - started) * 1000),
                  flush=True)

        def log_message(self, fmt, *args):      # keep stdout readable
            pass

    return Handler


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ckpt', help='checkpoint directory (pretrained_model)')
    parser.add_argument('--mock', action='store_true',
                        help='answer with a shape-correct synthetic block')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--host', default='0.0.0.0')
    args = parser.parse_args(argv)

    if not args.mock and not args.ckpt:
        parser.error('pass --ckpt or --mock')

    policy = Policy(args.ckpt or '', args.mock)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(policy))
    print('policy server on http://%s:%d/act   policy_id=%s'
          % (args.host, args.port, policy.policy_id), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nstopped', flush=True)
    finally:
        server.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
