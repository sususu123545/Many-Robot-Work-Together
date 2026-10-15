"""One bounded axis pulse; explicitly confirmed wheel-off-ground bench only."""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32MultiArray, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm-wheels-off-ground', action='store_true', required=True)
    parser.add_argument('--direction', choices=('forward', 'left', 'ccw'), default='forward')
    parser.add_argument('--calibration-only', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('migration_bench_forward_probe')
    state, encoders, wheels, odom = [], [], [], []
    node.create_subscription(String, '/chassis_control/status',
        lambda m: state.append(json.loads(m.data)), 10)
    node.create_subscription(Int32MultiArray, '/chassis_control/encoder_counts',
        lambda m: encoders.append((time.monotonic(), list(m.data))), 10)
    node.create_subscription(Int32MultiArray, '/chassis_control/wheel_command',
        lambda m: wheels.append((time.monotonic(), list(m.data))), 10)
    node.create_subscription(Odometry, '/odom', lambda m: odom.append(m), 10)
    velocity = node.create_publisher(Twist, '/cmd_vel', 1)
    stop = node.create_publisher(Empty, '/chassis_control/stop', 1)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)

    def halt():
        for _ in range(3):
            stop.publish(Empty())
            spin(.05)

    try:
        deadline = time.monotonic() + 12
        while (not state or len(encoders) < 20 or not odom
               or velocity.get_subscription_count() != 1) and time.monotonic() < deadline:
            spin(.1)
        assert state and len(encoders) >= 20 and odom, 'Hardware telemetry not ready'
        assert velocity.get_subscription_count() == 1, 'Expected exactly one command receiver'
        assert state[-1]['dry_run'] is False and state[-1]['enable_motion'] is True
        assert not state[-1]['estop'] and not state[-1]['hardware_fault'], 'Do not bypass an existing fault/estop'
        assert all(v == [0, 0, 0, 0] for _, v in wheels[-10:]), 'Nonzero output before pulse'
        assert encoders[-1][1] == encoders[-6][1], 'Wheels are not stationary before pulse'
        begin_counts, begin_odom = encoders[-1][1], odom[-1]

        command = Twist()
        if args.direction == 'forward':
            command.linear.x = .025
        elif args.direction == 'left':
            command.linear.y = .025
        else:
            command.angular.z = .08
        start = time.monotonic()
        print(f'BENCH_PULSE_BEGIN: {args.direction}, 1.0 second', flush=True)
        while time.monotonic() - start < 1.0:
            assert not state[-1]['hardware_fault'] and not state[-1]['estop']
            velocity.publish(command)
            spin(.05)
        stop_time = time.monotonic()
        halt()
        spin(1.0)
        assert not state[-1]['hardware_fault'], 'Hardware fault during pulse'
        stable = [v for t, v in wheels if t >= stop_time + .2]
        assert len(stable) >= 10 and all(v == [0, 0, 0, 0] for v in stable), 'Stop output not zero'
        assert encoders[-1][1] == encoders[-8][1], 'Encoder still moving after stop'
        delta = [((n - o + 2**31) % 2**32) - 2**31 for n, o in zip(encoders[-1][1], begin_counts)]
        dx = odom[-1].pose.pose.position.x - begin_odom.pose.pose.position.x
        dy = odom[-1].pose.pose.position.y - begin_odom.pose.pose.position.y
        q = odom[-1].pose.pose.orientation
        yaw = 2 * math.atan2(q.z, q.w)
        result = {'direction': args.direction, 'calibration_only': args.calibration_only,
                  'encoder_delta': delta, 'odom_dx': dx, 'odom_dy': dy, 'odom_yaw': yaw,
                  'wheel_outputs_seen': sorted({tuple(v) for t, v in wheels if start <= t <= stop_time}),
                  'final_wheel_output': wheels[-1][1], 'encoders_stopped': True,
                  'physical_translation_not_measured': True}
        print(json.dumps(result, indent=2), flush=True)
        assert all(abs(v) > 20 for v in delta), 'Some wheel encoders did not respond'
        if args.calibration_only:
            print('BENCH_RAW_CAPTURE=PASS; odometry axis not validated', flush=True)
        else:
            if args.direction == 'forward':
                assert dx > .001 and abs(dy) < max(.005, abs(dx) * .4) and abs(yaw) < .03, 'Forward odometry direction mismatch'
            elif args.direction == 'left':
                assert dy > .001 and abs(dx) < max(.005, abs(dy) * .4) and abs(yaw) < .03, 'Left odometry direction mismatch'
            else:
                assert yaw > .01 and math.hypot(dx, dy) < .005, 'Rotation odometry direction mismatch'
            print('BENCH_AXIS_PROBE=PASS', flush=True)
    finally:
        halt()  # Even a failed assertion must send immediate zero.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
