"""Closed-loop grasp: drive the IK tool point onto the measured ball centre.

Runs inside the armpi_ros2 core container. Every move is verified against the
servo feedback and retransmitted when the bus silently drops the packet
(observed on ~30-40 % of writes: "move written" but no joint motion).

Phases:
  1  coarse: swing to 50 mm above the ball
  2  servo:  measure -> delta = ball - tool -> move by delta (repeat)
  3  clamp:  close the gripper until the pads stall on the ball
  4  verify: lift 15 mm, re-measure; report whether the ball came along
"""
import json
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---- geometry (identical constants to armpi_ros2/arm_kinematics.py) ----
BASE_H, L1, L2, L3 = 0.064605, 0.10048, 0.094714, 0.16331
TPD = 1000.0 / 240.0
ARM = ('joint1', 'joint2', 'joint3', 'joint4')
CENTRE = {'joint1': 500, 'joint2': 500, 'joint3': 500, 'joint4': 500,
          'joint5': 500, 'r_joint': 700}
SIGN = {'joint1': 1, 'joint2': 1, 'joint3': -1, 'joint4': 1, 'joint5': 1, 'r_joint': 1}
LIMITS = {'joint1': (0, 1000), 'joint2': (125, 875), 'joint3': (0, 1000),
          'joint4': (0, 1000), 'joint5': (0, 1000), 'r_joint': (0, 700)}
MAX_STEP = 40


def tick_rad(name):
    return math.radians(180.0 if name == 'r_joint' else 240.0) / 1000.0


def to_rad(name, raw):
    return SIGN[name] * (raw - CENTRE[name]) * tick_rad(name)


def fk(raw):
    th4 = -(raw['joint3'] - 500) * 0.24
    th5 = (raw['joint2'] - 500) * 0.24
    th3 = (raw['joint4'] - 500) * 0.24
    th6 = (raw['joint1'] - 500) * 0.24
    q1, q2, q3 = (math.radians(th5), math.radians(th5 + th4),
                  math.radians(th5 + th4 + th3))
    radial = -(L1 * math.sin(q1) + L2 * math.sin(q2) + L3 * math.sin(q3))
    z = BASE_H + L1 * math.cos(q1) + L2 * math.cos(q2) + L3 * math.cos(q3)
    yaw = math.radians(th6)
    return -radial * math.sin(yaw), radial * math.cos(yaw), z


def ik(x, y, z, pitch_deg):
    radius = math.hypot(x, y)
    a = math.radians(pitch_deg)
    wr = radius + L3 * math.sin(a)
    wz = z - BASE_H - L3 * math.cos(a)
    ce = ((wr * wr + wz * wz - L1 * L1 - L2 * L2) / (2.0 * L1 * L2))
    if ce < -1.0 - 1e-9 or ce > 1.0 + 1e-9:
        return []
    ce = min(1.0, max(-1.0, ce))
    elbow = math.acos(ce)
    yaw = -math.degrees(math.atan2(x, y)) if radius > 1e-12 else 0.0
    out = []
    for t4 in (-elbow, elbow):
        sh = (math.atan2(wz, wr)
              - math.atan2(L2 * math.sin(t4), L1 + L2 * math.cos(t4)))
        t5 = sh - math.pi / 2.0
        t3 = a - t5 - t4
        deg = tuple(math.degrees(v) for v in (t3, t4, t5))
        tgt = {'joint1': round(round(120.0 + yaw) * TPD),
               'joint2': round(deg[2] * TPD + 500.0),
               'joint3': round(-deg[1] * TPD + 500.0),
               'joint4': round(deg[0] * TPD + 500.0)}
        if all(LIMITS[n][0] <= v <= LIMITS[n][1] for n, v in tgt.items()):
            out.append(tgt)
    return out


class Grasp(Node):
    def __init__(self):
        super().__init__('grasp_closed_loop')
        self.status = {}
        self.ball = None
        self.traj = self.create_publisher(JointTrajectory, '/arm_controller/command', 10)
        self.create_subscription(String, '/arm_controller/status',
                                 lambda m: self.status.update(json.loads(m.data)), 10)
        self.create_subscription(PointStamped, '/grasp/target_base',
                                 lambda m: setattr(self, 'ball', m), 10)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def raw(self):
        joints = self.status.get('joints', {})
        return {n: joints.get(n, {}).get('raw') for n in CENTRE}

    def temp(self, name):
        return self.status.get('joints', {}).get(name, {}).get('temperature')

    def wait_status(self, seconds=10.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.status:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ----- one command with verify + retransmit -----
    # The servos stop inside their own positioning deadband (measured 8-12
    # ticks), so "arrived" must be looser than that or every successful move
    # looks like a dropped packet.
    def send(self, targets, duration=2.0, tries=4, tolerance=14):
        for attempt in range(1, tries + 1):
            msg = JointTrajectory()
            msg.joint_names = list(targets)
            point = JointTrajectoryPoint()
            point.positions = [to_rad(n, v) for n, v in targets.items()]
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int(round((duration - int(duration)) * 1e9))
            msg.points = [point]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.traj.publish(msg)
            self.spin_for(duration + 0.9)
            now = self.raw()
            err = max(abs((now.get(n) or 0) - v) for n, v in targets.items())
            if err <= tolerance:
                return True, attempt, err
        return False, tries, err

    def move_tool(self, target, pitch_pref=(-152, -148, -144, -140, -136)):
        """Move the IK tool point to `target` in <=MAX_STEP sub-steps."""
        sols = []
        for pitch in pitch_pref:
            sols = ik(target[0], target[1], target[2], pitch)
            if sols:
                break
        if not sols:
            return 'NO_IK'
        start = self.raw()
        best = min(sols, key=lambda s: max(abs(s[n] - start[n]) for n in ARM))
        cur = {n: start[n] for n in ARM}
        n_steps = max(1, int(math.ceil(max(abs(best[n] - cur[n]) for n in ARM) / MAX_STEP)))
        problems = 0
        for i in range(1, n_steps + 1):
            s = i / n_steps
            step = {n: int(round(cur[n] + (best[n] - cur[n]) * s)) for n in ARM}
            step['joint5'] = int(self.raw().get('joint5') or 495)
            ok, tries, err = self.send(step)
            if not ok:
                problems += 1
                # keep going: the servo loop re-measures and corrects anyway
        return 'OK' if problems == 0 else 'OK_WITH_%d_RETRIES' % problems

    def measure(self, timeout=8.0):
        """Fresh ball sample in arm_base (wait for a new message after a move)."""
        self.ball = None
        end = time.monotonic() + timeout
        while time.monotonic() < end and self.ball is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.ball is None:
            return None
        p = self.ball.point
        return (p.x, p.y, p.z)

    def clamp(self, stop_raw, step=70, tries=3):
        """Close the jaws; report contact when a step settles far short of its
        target (a normal move only falls short by the servo deadband)."""
        cur = self.raw()['r_joint']
        while cur < stop_raw:
            nxt = min(stop_raw, cur + step)
            self.send({'r_joint': nxt}, duration=1.5, tries=tries, tolerance=12)
            new = self.raw()['r_joint']
            if new < nxt - 22:
                return new, True
            if new - cur < 12:
                return new, True
            cur = new
        return cur, False


def main():
    launch = subprocess.Popen(
        ['ros2', 'launch', 'armpi_ros2', 'grasp_vision.launch.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(9)
    rclpy.init()
    node = Grasp()
    log = []
    result = {}
    try:
        node.wait_status()
        st = node.status
        result['initial'] = {'estop': st.get('estop'), 'health': st.get('health_state'),
                             'arm_on': st.get('enable_arm_motion'),
                             'grip_on': st.get('enable_gripper_motion'),
                             'raw': node.raw()}
        if st.get('estop') or not st.get('enable_arm_motion') or not st.get('enable_gripper_motion'):
            print(json.dumps({'error': 'gates not open', **result}, sort_keys=True))
            return

        ball = node.measure()
        if ball is None:
            print(json.dumps({'error': 'no ball detection'}, sort_keys=True))
            return
        result['ball_0'] = [round(v, 4) for v in ball]

        # phase 1: coarse, 50 mm above the ball
        tool = fk(node.raw())
        result['tool_0'] = [round(v, 4) for v in tool]
        res = node.move_tool((ball[0], ball[1], ball[2] + 0.050))
        log.append(['coarse', res])

        # phase 2: servo the tool onto the ball, one measured delta at a time
        for iteration in range(1, 11):
            now_ball = node.measure()
            if now_ball is None:
                log.append(['measure_%d' % iteration, 'NO_BALL'])
                break
            tool = fk(node.raw())
            dx, dy, dz = (now_ball[i] - tool[i] for i in range(3))
            gap = math.sqrt(dx * dx + dy * dy + dz * dz)
            log.append(['delta_%d' % iteration,
                        {'tool': [round(v, 4) for v in tool],
                         'ball': [round(v, 4) for v in now_ball],
                         'delta_mm': [round(dx * 1000, 1), round(dy * 1000, 1),
                                      round(dz * 1000, 1)],
                         'gap_mm': round(gap * 1000, 1)}])
            if gap < 0.006:
                break
            # clamp the correction so one iteration never sweeps blindly
            scale = min(1.0, 0.025 / gap)
            target = (tool[0] + dx * scale, tool[1] + dy * scale, tool[2] + dz * scale)
            res = node.move_tool(target)
            log.append(['servo_%d' % iteration, res])
            if res != 'OK':
                break

        # phase 3: clamp
        stop, touched = node.clamp(430)
        log.append(['clamp', {'r_final': stop, 'touched_object': touched,
                              'temp': node.temp('r_joint')}])

        # phase 4: lift and compare the ball offset relative to the tool
        before = fk(node.raw())
        res = node.move_tool((before[0], before[1], before[2] + 0.018))
        log.append(['lift', res])
        after_ball = node.measure()
        after_tool = fk(node.raw())
        if after_ball is not None:
            gap_after = math.sqrt(sum((after_ball[i] - after_tool[i]) ** 2 for i in range(3)))
            log.append(['verify', {'tool': [round(v, 4) for v in after_tool],
                                   'ball': [round(v, 4) for v in after_ball],
                                   'gap_mm': round(gap_after * 1000, 1)}])
            result['verdict'] = 'HELD' if gap_after < 0.020 else 'NOT_HELD'
        else:
            result['verdict'] = 'BALL_LOST_FROM_VIEW'
        result['final_raw'] = node.raw()
        result['temperatures'] = {n: node.temp(n) for n in CENTRE}
    finally:
        result['log'] = log
        print(json.dumps(result, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        launch.terminate()
        for pattern in ('grasp_vision.launch.py', 'red_ball_detector',
                        'grasp_target_transform', 'arm_camera_transform'):
            subprocess.run(['pkill', '-f', pattern], check=False)


if __name__ == '__main__':
    main()
