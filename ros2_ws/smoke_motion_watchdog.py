"""Native ROS 2 command-safety probe; REFUSE any real hardware mode."""
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Int32MultiArray, String
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('migration_watchdog_probe')
    status, wheels = [], []
    node.create_subscription(String, '/chassis_control/status',
        lambda m: status.append(json.loads(m.data)), 10)
    node.create_subscription(Int32MultiArray, '/chassis_control/wheel_command',
        lambda m: wheels.append((time.monotonic(), list(m.data))), 10)
    velocity = node.create_publisher(Twist, '/cmd_vel', 1)
    stop = node.create_publisher(Empty, '/chassis_control/stop', 1)
    estop = node.create_publisher(Bool, '/safety/estop', 1)
    reset = node.create_client(Trigger, '/chassis_control/reset_estop')

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)

    def move(seconds=.30):
        before = len(wheels)
        command = Twist()
        command.linear.x = .02
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            velocity.publish(command)
            spin(.05)
        return wheels[before:]

    def stationary_after(sent, grace=.15):
        recent = [v for t, v in wheels if t >= sent + grace]
        assert len(recent) >= 5 and all(v == [0, 0, 0, 0] for v in recent), recent[-20:]

    results = []
    try:
        deadline = time.monotonic() + 10
        while (not status or velocity.get_subscription_count() != 1
               or stop.get_subscription_count() != 1) and time.monotonic() < deadline:
            spin(.1)
        assert status, 'Chassis status did not become ready'
        assert status[-1]['dry_run'] is True, 'Refusing hardware mode'
        assert status[-1]['enable_motion'] is True and not status[-1]['hardware_fault']
        assert velocity.get_subscription_count() == 1 and stop.get_subscription_count() == 1

        assert any(any(v) for _, v in move()), 'No simulated command output'
        sent = time.monotonic()
        spin(.8)
        stationary_after(sent, grace=.5)
        results.append('command_timeout_zero')

        move()
        sent = time.monotonic()
        stop.publish(Empty())
        spin(.5)
        stationary_after(sent)
        results.append('immediate_stop_stays_zero')

        move()
        invalid = Twist()
        invalid.linear.x = float('nan')
        sent = time.monotonic()
        velocity.publish(invalid)
        spin(.5)
        stationary_after(sent)
        results.append('nonfinite_command_cancels_motion')

        move()
        estop.publish(Bool(data=True))
        spin(.2)
        sent = time.monotonic()
        estop.publish(Bool(data=False))
        move()
        spin(1.1)
        stationary_after(sent)
        assert status[-1]['estop'] is True
        results.append('estop_false_does_not_unlock')

        assert reset.wait_for_service(timeout_sec=2)
        future = reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        assert future.done() and future.result().success
        sent = time.monotonic()
        spin(.5)
        stationary_after(sent)
        assert any(any(v) for _, v in move()), 'Fresh simulated command rejected after reset'
        stop.publish(Empty())
        spin(.3)
        results.append('reset_does_not_replay_old_command')
        print(json.dumps({'success': True, 'simulation_only': True,
                          'checks': results, 'final_output': wheels[-1][1]}, indent=2))
    finally:
        # Allowed only after verifying this is a simulation; never reset a real estop.
        if status and status[-1]['dry_run'] is True:
            stop.publish(Empty())
            spin(.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
