"""Drive a step-and-stare sweep until the target is actually visible.

Starts only on an explicit service call. A robot that begins sweeping on its own
at boot is a hazard, and this node has no way to know whether a person is
standing next to it.

Motion gates are respected, not bypassed:
* ``/chassis_control/status`` must report ``enable_motion: true``; until then the
  node waits with the wheels stopped and says so;
* ``/safety/estop`` aborts the sweep immediately;
* ``/cmd_vel`` is published continuously while running because the chassis lease
  expires 0.35 s after the last command -- a node that publishes once and waits
  is a node whose robot stops in the middle of a turn.

The camera on this robot is mounted on the arm (``arm_camera_tf`` publishes
``arm_base -> camera`` from joint feedback). This node turns the base only. If
the arm is folded so that the camera stares at the chassis, the sweep will
correctly report "not found" forever, so check the view before blaming the ball.
"""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robot_agent.search import ScanPlanner, ScanState


class SearchScanNode(Node):
    def __init__(self):
        super().__init__('vla_search_scan')
        defaults = {
            'step_deg': 45.0,
            'max_sweep_deg': 360.0,
            'spin_speed': 0.18,
            'settle_s': 0.8,
            'observe_s': 2.0,
            'confirm_s': 0.4,
            'max_duration_s': 240.0,
            'direction': 1.0,
            'detection_topic': '/grasp/red_ball/detection',
            'detection_timeout_s': 0.6,
            'cmd_vel_topic': '/cmd_vel',
            'chassis_status_topic': '/chassis_control/status',
            'status_topic': '/vla/search_status',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name)

        self.planner = ScanPlanner(
            step_rad=math.radians(p('step_deg').value),
            max_sweep_rad=math.radians(p('max_sweep_deg').value),
            spin_speed=p('spin_speed').value,
            settle_s=p('settle_s').value,
            observe_s=p('observe_s').value,
            confirm_s=p('confirm_s').value,
            max_duration_s=p('max_duration_s').value,
            direction=p('direction').value)
        self.detection_timeout = float(p('detection_timeout_s').value)

        self.detection_seen_at = None
        self.detection_valid = False
        self.detection_payload = {}
        self.chassis_status = {}
        self.chassis_status_at = None
        self.estop = False
        self.stop_messages_left = 0

        self.cmd_pub = self.create_publisher(Twist, str(p('cmd_vel_topic').value), 1)
        self.status_pub = self.create_publisher(String, str(p('status_topic').value), 10)
        self.create_subscription(String, str(p('detection_topic').value),
                                 self.on_detection, 5)
        self.create_subscription(String, str(p('chassis_status_topic').value),
                                 self.on_chassis_status, 10)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, 10)
        self.create_service(Trigger, '/vla/search/start', self.on_start)
        self.create_service(Trigger, '/vla/search/stop', self.on_stop_service)

        self.create_timer(0.05, self.on_tick)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            'Search ready: %.0f deg steps, %.2f rad/s, %.0f deg sweep. '
            'Call /vla/search/start to begin.'
            % (p('step_deg').value, p('spin_speed').value, p('max_sweep_deg').value))

    # -- inputs ------------------------------------------------------------

    def on_detection(self, msg):
        self.detection_seen_at = self.now()
        try:
            self.detection_payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.detection_payload = {}
            self.detection_valid = False
            return
        self.detection_valid = bool(self.detection_payload.get('valid', False))

    def on_chassis_status(self, msg):
        try:
            self.chassis_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.chassis_status = {}
        self.chassis_status_at = self.now()

    def on_estop(self, msg):
        if msg.data:
            self.estop = True

    # -- readiness ---------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def detection_fresh_valid(self):
        if not self.detection_valid or self.detection_seen_at is None:
            return False
        return self.now() - self.detection_seen_at <= self.detection_timeout

    def blocked_reason(self):
        """Why the sweep may not move right now, or '' when it may."""
        if self.estop:
            return '/safety/estop is latched'
        if not self.chassis_status:
            return 'no /chassis_control/status yet'
        if self.now() - (self.chassis_status_at or 0.0) > 2.0:
            return 'chassis status is stale'
        if self.chassis_status.get('estop'):
            return 'chassis motion is latched'
        if self.chassis_status.get('hardware_fault'):
            return 'chassis hardware fault'
        if not self.chassis_status.get('enable_motion'):
            return 'chassis motion permission is OFF'
        return ''

    # -- services ----------------------------------------------------------

    def on_start(self, _request, response):
        blocked = self.blocked_reason()
        if blocked:
            response.success = False
            response.message = 'Refusing to start: ' + blocked
            return response
        self.planner.start(self.now())
        self.stop_messages_left = 0
        response.success = True
        response.message = ('Sweep started: %.0f deg steps, %.2f rad/s'
                            % (self.get_parameter('step_deg').value,
                               self.get_parameter('spin_speed').value))
        self.get_logger().info(response.message)
        return response

    def on_stop_service(self, _request, response):
        was_running = self.planner.running
        self.planner.stop()
        self.stop_messages_left = 5
        response.success = True
        response.message = 'Sweep stopped' if was_running else 'Nothing was running'
        return response

    # -- control loop ------------------------------------------------------

    def on_tick(self):
        now = self.now()
        if self.stop_messages_left > 0:
            self.publish_twist(0.0, 0.0)
            self.stop_messages_left -= 1
            return
        if not self.planner.running:
            return

        blocked = self.blocked_reason()
        if blocked:
            # Do not advance the planner; the operator may still grant motion.
            self.publish_twist(0.0, 0.0)
            return

        command = self.planner.tick(now, self.detection_fresh_valid())
        if command.kind == 'spin':
            self.publish_twist(0.0, command.wz)
        else:
            self.publish_twist(0.0, 0.0)

        if command.kind == 'found':
            self.stop_messages_left = 5
            self.get_logger().info('Target found after turning %.0f deg'
                                   % math.degrees(command.travelled_rad))
        elif command.kind == 'stop' and command.state in (ScanState.EXHAUSTED,
                                                          ScanState.ABORTED):
            self.stop_messages_left = 5
            self.get_logger().warning('Search ended (%s) after %.0f deg: %s'
                                      % (command.state,
                                         math.degrees(command.travelled_rad),
                                         command.detail))

    def publish_twist(self, linear, angular):
        message = Twist()
        message.linear.x = float(linear)
        message.linear.y = 0.0
        message.angular.z = float(angular)
        self.cmd_pub.publish(message)

    # -- status ------------------------------------------------------------

    def publish_status(self):
        snapshot = self.planner.snapshot()
        payload = {
            'running': self.planner.running,
            'state': snapshot['state'],
            'travelled_deg': snapshot['travelled_deg'],
            'sweep_target_deg': snapshot['sweep_target_deg'],
            'steps_done': snapshot['steps_done'],
            'abort_reason': snapshot['abort_reason'],
            'blocked_reason': self.blocked_reason(),
            'detection_valid': self.detection_fresh_valid(),
            'detection_age_s': (None if self.detection_seen_at is None
                                else round(self.now() - self.detection_seen_at, 3)),
            'detection_reason': self.detection_payload.get('reason'),
            'point_camera_m': self.detection_payload.get('point_camera_m'),
            'center_px': (None if not self.detection_payload.get('detection')
                          else [self.detection_payload['detection'].get('center_x'),
                                self.detection_payload['detection'].get('center_y')]),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))


def main(argv=None):
    rclpy.init(args=argv)
    node = SearchScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
