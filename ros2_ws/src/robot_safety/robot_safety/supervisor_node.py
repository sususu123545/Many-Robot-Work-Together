"""Publish a liveness verdict for the links this robot depends on.

The verdict goes to ``/vla/safety_estop`` by default, **not** to the existing
``/safety/estop`` that the chassis and arm controllers already subscribe to.
That is a deliberate default: wiring a new supervisor into the hardware stop is
a decision that should be made once, after watching the supervisor behave, not
inherited from a launch file. Point ``estop_topic`` at ``/safety/estop`` when
that decision has been made and verified.

The node is intentionally type-flexible: a watchdog that can only watch one
message type gets bypassed the first time someone needs to watch something else.
"""

import json

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robot_safety.watchdog import SafetyLatch, Watchdog

#: Message types a source may declare, and the QoS a subscription for it needs.
#: Sensor streams are best-effort because that is how their publishers offer
#: them; the rest are reliable because that is the default everywhere else.
_TYPES = {
    'joint_state': (JointState, 10),
    'string': (String, 10),
    'odom': (Odometry, qos_profile_sensor_data),
    'twist': (Twist, qos_profile_sensor_data),
    'scan': (LaserScan, qos_profile_sensor_data),
    'image': (Image, qos_profile_sensor_data),
    'compressed_image': (CompressedImage, qos_profile_sensor_data),
    'camera_info': (CameraInfo, qos_profile_sensor_data),
    'bool': (Bool, 10),
}

_DEFAULTS = {
    'source_names': ['joint_states', 'arm_status'],
    'source_topics': ['/joint_states', '/arm_controller/status'],
    'source_types': ['joint_state', 'string'],
    'source_timeouts': [1.5, 2.0],
    'source_required': [True, True],
    'estop_topic': '/vla/safety_estop',
    'publish_to_hardware_estop': False,
    'latch_on_missing': False,
    'recovery_s': 2.0,
    'startup_grace_s': 10.0,
    'status_topic': '/safety/watchdog_status',
}


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__('vla_safety_supervisor')
        for name, value in _DEFAULTS.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value

        names = list(p('source_names'))
        topics = list(p('source_topics'))
        types = list(p('source_types'))
        timeouts = list(p('source_timeouts'))
        required = list(p('source_required'))
        lengths = {len(names), len(topics), len(types), len(timeouts), len(required)}
        if len(lengths) != 1:
            raise ValueError('source_* parameter lists must all have the same length')

        self.watchdog = Watchdog({
            name: {'timeout': timeouts[index], 'required': required[index]}
            for index, name in enumerate(names)})
        self.latch = SafetyLatch(self.watchdog, latch_on_missing=bool(p('latch_on_missing')),
                                 recovery_s=p('recovery_s'),
                                 startup_grace_s=p('startup_grace_s'))

        for index, name in enumerate(names):
            kind = types[index]
            if kind not in _TYPES:
                raise ValueError('Unknown source type %r for %r; known: %s'
                                 % (kind, name, ', '.join(sorted(_TYPES))))
            message_type, qos = _TYPES[kind]
            self.create_subscription(message_type, topics[index],
                                     self._beat_callback(name), qos)

        estop_topic = str(p('estop_topic'))
        if bool(p('publish_to_hardware_estop')) and estop_topic != '/safety/estop':
            raise ValueError('publish_to_hardware_estop requires estop_topic="/safety/estop"; '
                             'refusing a configuration that looks like it stops the robot '
                             'but does not')
        self.estop_pub = self.create_publisher(Bool, estop_topic, 10)
        self.status_pub = self.create_publisher(String, str(p('status_topic')), 10)
        self.create_service(Trigger, '/safety/clear_estop', self.on_clear)

        self.create_timer(0.1, self.on_evaluate)
        self.create_timer(0.5, self.on_publish_estop)
        self.create_timer(1.0, self.on_publish_status)
        self.estop_topic = estop_topic
        self.last_report = {}
        self.get_logger().info(
            'Watching %d source(s); estop -> %s (hardware stop: %s)'
            % (len(names), estop_topic,
               'WIRED' if bool(p('publish_to_hardware_estop')) else 'not wired'))
        if not bool(p('publish_to_hardware_estop')):
            self.get_logger().warning(
                'This supervisor does NOT stop the robot by itself. Set '
                'publish_to_hardware_estop:=true and estop_topic:=/safety/estop '
                'once you have watched it behave.')

    def _beat_callback(self, name):
        def callback(_message):
            self.watchdog.beat(name, self.now())
        return callback

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_evaluate(self):
        self.last_report = self.latch.evaluate(self.now())
        if self.last_report.get('latched') and not getattr(self, '_latch_logged', False):
            self._latch_logged = True
            self.get_logger().error('Safety latch: %s'
                                    % self.last_report.get('latch_reason'))
        if not self.last_report.get('latched'):
            self._latch_logged = False

    def on_publish_estop(self):
        if not self.last_report:
            return
        self.estop_pub.publish(Bool(data=bool(self.last_report.get('latched', False))))

    def on_publish_status(self):
        if not self.last_report:
            return
        report = dict(self.last_report)
        detail = report.pop('detail', {})
        report['ages'] = {name: (None if age == float('inf') else round(age, 3))
                          for name, age in detail.items()}
        report['estop_topic'] = self.estop_topic
        report['counts'] = dict(self.watchdog.count)
        self.status_pub.publish(String(data=json.dumps(report)))

    def on_clear(self, _request, response):
        ok = self.latch.clear(self.now())
        response.success = ok
        response.message = ('Safety latch cleared' if ok else
                            'Refused: latch is either not set or the watched links '
                            'have not been healthy for %.1f s' % self.latch.recovery_s)
        if ok:
            self.get_logger().warning('Safety latch cleared by service request')
        return response


def main(argv=None):
    rclpy.init(args=argv)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
