"""Single I2C owner, native rclpy base driver, bench testing only initially."""
import json
import struct
import time
from contextlib import closing

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Int32MultiArray, String
from std_srvs.srv import Trigger

from .motion import Geometry, MotionLease, finite, wheel_commands


class ChassisNode(Node):
    def __init__(self):
        super().__init__('chassis_control')
        defaults = {'dry_run': True, 'enable_motion': False, 'i2c_bus': 1,
                    'i2c_address': 0x34, 'motor_type': 3, 'encoder_rate': 20.0,
                    'command_timeout': 0.35, 'max_linear': 0.08,
                    'max_angular': 0.20, 'max_pulse': 100,
                    'a': 0.110, 'b': 0.0975, 'wheel_diameter': 0.0965,
                    'pulses_per_rev': 7832.0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.dry_run, self.enable_motion = p('dry_run'), p('enable_motion')
        self.geometry = Geometry(*(float(p(k)) for k in
                                   ('a', 'b', 'wheel_diameter', 'pulses_per_rev')))
        self.lease = MotionLease(p('command_timeout'), p('max_linear'), p('max_angular'))
        self.max_pulse = p('max_pulse')
        wheel_commands(0, 0, 0, self.geometry, self.max_pulse)  # validate
        if not finite(p('encoder_rate')) or p('encoder_rate') <= 0:
            raise ValueError('encoder_rate must be positive')
        self.hardware_fault = False
        self.bus, self.address = p('i2c_bus'), p('i2c_address')
        self.last_written = None
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/chassis_control/encoder_counts', 10)
        self.output_pub = self.create_publisher(Int32MultiArray, '/chassis_control/wheel_command', 10)
        self.status_pub = self.create_publisher(String, '/chassis_control/status', 10)
        if not self.dry_run:
            from smbus2 import SMBus
            self.SMBus = SMBus
            with closing(self.SMBus(self.bus)) as bus:
                bus.write_i2c_block_data(self.address, 20, [p('motor_type')])
        self.write_zero()
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
        self.create_subscription(Empty, '/chassis_control/stop', self.on_stop, 1)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, 1)
        self.create_service(Trigger, '/chassis_control/reset_estop', self.reset_estop)
        self.create_timer(0.02, self.tick)
        self.create_timer(1.0 / p('encoder_rate'), self.read_encoders)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'dry_run={self.dry_run}, enable_motion={self.enable_motion}; no arm commands')

    def write(self, pulses):
        if not self.dry_run:
            with closing(self.SMBus(self.bus)) as bus:
                bus.write_i2c_block_data(self.address, 51, [v & 0xff for v in pulses])
        self.last_written = list(pulses)
        if rclpy.ok():
            self.output_pub.publish(Int32MultiArray(data=list(pulses)))

    def write_zero(self):
        self.write([0, 0, 0, 0])

    def fault(self, error):
        self.hardware_fault = True
        self.lease.stop(latch=True)
        self.get_logger().error(f'I2C fault; emergency latched: {error}', throttle_duration_sec=2.0)
        try:
            self.write_zero()
        except OSError:
            # Broken power/I2C cannot be repaired by publishing software zero.
            self.get_logger().error('Unable to confirm motor zero; use physical emergency stop', throttle_duration_sec=2.0)

    def on_command(self, msg):
        valid = self.lease.accept(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())
        if not valid or not any(self.lease.command):
            self.on_stop(None)

    def on_stop(self, _msg):
        self.lease.stop()
        try:
            self.write_zero()  # No acceleration ramp or delayed worker after stop.
        except OSError as error:
            self.fault(error)

    def on_estop(self, msg):
        if msg.data:
            self.lease.stop(latch=True)
            self.on_stop(None)
        # False does not clear emergency automatically.

    def reset_estop(self, _request, response):
        try:
            self.write_zero()
            if not self.dry_run:
                with closing(self.SMBus(self.bus)) as bus:
                    raw = bus.read_i2c_block_data(self.address, 60, 16)
                    if len(raw) != 16:
                        raise OSError('Incomplete encoder reply; emergency stays latched')
            self.hardware_fault = False
            self.lease.reset()
            response.success = True
            response.message = 'Reset; awaiting NEW velocity command'
        except OSError as error:
            response.success = False
            response.message = str(error)
        return response

    def tick(self):
        command = self.lease.output(time.monotonic())
        if not self.enable_motion or self.hardware_fault:
            command = (0.0, 0.0, 0.0)
        pulses = wheel_commands(*command, self.geometry, self.max_pulse)
        try:
            # Periodically write even zero so an I2C failure is observable.
            self.write(pulses)
        except OSError as error:
            self.fault(error)

    def read_encoders(self):
        try:
            if self.dry_run:
                counts = [0] * 4  # Explicitly simulated, never fabricated displacement.
            else:
                with closing(self.SMBus(self.bus)) as bus:
                    raw = bus.read_i2c_block_data(self.address, 60, 16)
                counts = list(struct.unpack('<4i', bytes(raw)))
            self.encoder_pub.publish(Int32MultiArray(data=counts))
        except (OSError, struct.error) as error:
            self.fault(error)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'dry_run': self.dry_run, 'enable_motion': self.enable_motion,
            'estop': self.lease.estop, 'hardware_fault': self.hardware_fault,
            'wheel_command': self.last_written})))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ChassisNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.on_stop(None)
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
