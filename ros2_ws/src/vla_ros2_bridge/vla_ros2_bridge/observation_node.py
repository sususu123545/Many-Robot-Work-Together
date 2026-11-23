"""Publish ``/vla/observation`` from the robot's existing topics.

Thin ROS 2 shell: every decision (which joints are required, whether the image
is fresh enough, how ticks map to radians and TCP pose) lives in
``vla_ros2_bridge.observation``, which is pure Python and unit-tested.  This
node only converts messages to plain values and back.

It deliberately does NOT invent a timestamp when ``/joint_states`` arrives with
a zero stamp.  A fabricated "now" would make a stale joint frame look
simultaneous with the image, which is exactly the corruption the builder is
there to prevent.

No motion permission is requested here.  This node only publishes.
"""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Header, String
from vla_interfaces.msg import VlaObservation

from vla_controller import spec
from vla_ros2_bridge.observation import (ObservationBuilder, ObservationError,
                                         seconds)


def time_from_seconds(value):
    value = float(value)
    whole = math.floor(value)
    return Time(sec=int(whole), nanosec=int(round((value - whole) * 1e9)))


def ticks_from_joint_state(message):
    """Radian JointState -> servo ticks, refusing an encoder that is lying.

    Same rule as the executor's ``_ticks_from_joint_state``: an out-of-range
    reading is evidence of a fault, not noise to be clamped.  Kept local so the
    observation side does not import the executor's private helpers.
    """
    raw = {}
    for name, radians in zip(message.name, message.position):
        if name not in spec.ARM_PI_JOINTS:
            continue
        if not math.isfinite(radians):
            return {}, 0.0
        try:
            _, tick = spec.radians_to_tick(name, radians)
        except ValueError:
            return {}, 0.0
        raw[name] = tick
    return raw, seconds(message.header.stamp)


class ObservationNode(Node):

    def __init__(self):
        super().__init__('vla_observation_node')
        self.declare_parameter('image_topic', '/rear_camera/image_raw/compressed')
        self.declare_parameter('instruction_topic', '/vla/instruction')
        self.declare_parameter('observation_topic', '/vla/observation')
        self.declare_parameter('max_skew_s', 0.75)
        self.declare_parameter('require_wrist_image', True)
        self.declare_parameter('frame_id', spec.ACTION_FRAME)
        self.declare_parameter('instruction', '')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.builder = ObservationBuilder(
            max_skew_s=p('max_skew_s'),
            frame_id=p('frame_id'),
            require_wrist_image=p('require_wrist_image'))
        self.instruction = str(p('instruction') or '')

        self.publisher = self.create_publisher(VlaObservation,
                                               p('observation_topic'), 10)
        self.create_subscription(JointState, '/joint_states',
                                 self.on_joint_state, 10)
        self.create_subscription(CompressedImage, p('image_topic'),
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, p('instruction_topic'),
                                 self.on_instruction, 10)

        self.latest_ticks = {}
        self.joint_stamp = 0.0
        self.latest_image = b''
        self.image_format = 'jpeg'
        self.image_stamp = 0.0
        self.last_published = None
        self.refusals = 0
        self.get_logger().info('VLA observation node ready (image=%s)'
                               % p('image_topic'))

    # -- inputs ------------------------------------------------------------

    def on_instruction(self, message):
        self.instruction = str(message.data)
        self.get_logger().info('Instruction set: %r' % self.instruction)

    def on_image(self, message):
        self.latest_image = bytes(message.data)
        self.image_format = str(message.format or 'jpeg')
        self.image_stamp = seconds(message.header.stamp)

    def on_joint_state(self, message):
        ticks, stamp = ticks_from_joint_state(message)
        if not ticks or stamp <= 0.0:
            self.refusals += 1
            if self.refusals % 20 == 1:
                self.get_logger().warn(
                    'Ignoring /joint_states: unusable tick conversion or zero '
                    'stamp (refusals=%d)' % self.refusals)
            return
        self.latest_ticks = ticks
        self.joint_stamp = stamp
        self.try_publish()

    # -- output ------------------------------------------------------------

    def try_publish(self):
        from vla_ros2_bridge.observation import SensorSnapshot

        snapshot = SensorSnapshot(
            joint_ticks=dict(self.latest_ticks), joint_stamp=self.joint_stamp,
            wrist_image=self.latest_image, wrist_image_format=self.image_format,
            wrist_stamp=self.image_stamp, instruction=self.instruction)
        try:
            observation = self.builder.build(snapshot)
        except ObservationError as error:
            self.refusals += 1
            if self.refusals % 20 == 1:
                self.get_logger().warn('Observation refused: %s (refusals=%d)'
                                       % (error, self.refusals))
            return
        key = (self.joint_stamp, self.image_stamp)
        if key == self.last_published:
            return
        self.last_published = key
        self.publisher.publish(self.to_message(observation))

    def to_message(self, observation):
        data = observation.as_msg_kwargs()
        header = Header()
        header.stamp = time_from_seconds(data['header']['stamp'])
        header.frame_id = data['header']['frame_id']

        message = VlaObservation()
        message.header = header
        message.instruction = data['instruction']
        message.robot_config_id = data['robot_config_id']
        position = [float(v) for v in data['tcp_position_m']]
        orientation = [float(v) for v in data['tcp_orientation']]
        # rosidl-generated Python messages are keyword-only; positional
        # construction silently fails on some distributions.
        message.tcp_position_m = Point(x=position[0], y=position[1], z=position[2])
        message.tcp_orientation = Quaternion(x=orientation[0], y=orientation[1],
                                             z=orientation[2], w=orientation[3])
        message.joint_names = list(data['joint_names'])
        message.joint_positions_rad = [float(v) for v in data['joint_positions_rad']]
        message.gripper_opening = float(data['gripper_opening'])
        message.wrist_image_stamp = time_from_seconds(data['wrist_image_stamp'])
        wrist = CompressedImage()
        wrist.format = data['wrist_image']['format']
        wrist.data = bytes(data['wrist_image']['data'])
        message.wrist_image = wrist
        # No scene camera on this platform: empty data, not a fabricated frame.
        message.front_image = CompressedImage()
        message.front_image_stamp = Time(sec=0, nanosec=0)
        return message


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = ObservationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
