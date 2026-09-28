#!/usr/bin/python3
# coding=utf8
# Date:2022/03/30
import sys
import math
import json
import struct
import time
from contextlib import closing
import rospy
# The base ROS image already ships the kernel-backed ``smbus`` module, while
# some development images use the API-compatible ``smbus2`` package.  Keep the
# chassis node startable in both environments; both expose SMBus and the
# block read/write methods used below.
try:
    import smbus2
except ImportError:
    import smbus as smbus2
from threading import Lock, Thread, current_thread
from std_msgs.msg import *
from chassis_control.msg import *
from armpi_pro import misc

ENCODER_MOTOR_MODULE_ADDRESS = 0x34
MOTOR_ENCODER_TOTAL_ADDR = 60
ENCODER_CHANNELS = 4
DEFAULT_ENCODER_RATE = 20.0
WALL_FOLLOW_ACTIVE_STATES = frozenset((
    'FOLLOW', 'TURN', 'LOST', 'RECOVER', 'WAIT_OBSTACLE', 'ESCAPE'))
DEFAULT_WALL_FOLLOW_WATCHDOG = 1.5

th = None
slow_en = True
motion_generation = 0
motion_lock = Lock()
wall_follow_active = False
wall_follow_last_status = 0.0
wall_follow_watchdog_timeout = DEFAULT_WALL_FOLLOW_WATCHDOG


class EncoderMotorController:
    def __init__(self, i2c_port, motor_type=3):
        self.i2c_port = i2c_port
        self._i2c_lock = Lock()
        with self._i2c_lock, closing(smbus2.SMBus(self.i2c_port)) as bus:
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 20, [motor_type, ])

    def set_speed(self, speed, motor_id=None, offset=0):
        global th
        # 通过IIC发布控制信息到电机驱动板
        with self._i2c_lock, closing(smbus2.SMBus(self.i2c_port)) as bus:
            try:
                if motor_id is None:
                    bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 51 + offset, speed)
                else:
                    if 0 < motor_id <= 4:
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + motor_id, [speed, ])
                    else:
                        raise ValueError("Invalid motor id")

            except Exception as e:
                th = None
                print(e)

    def read_encoder_counts(self):
        """Read the four signed 32-bit cumulative encoder counters.

        The Hiwonder encoder motor module exposes the total counters at
        register 60, four little-endian int32 values (16 bytes total).
        Keep the read on the same lock as motor writes: the control callbacks
        and the encoder timer run in different threads and the I2C device does
        not tolerate overlapping transactions reliably.
        """
        with self._i2c_lock, closing(smbus2.SMBus(self.i2c_port)) as bus:
            raw = bus.read_i2c_block_data(
                ENCODER_MOTOR_MODULE_ADDRESS,
                MOTOR_ENCODER_TOTAL_ADDR,
                ENCODER_CHANNELS * 4,
            )
        if len(raw) != ENCODER_CHANNELS * 4:
            raise IOError('encoder register returned %d bytes, expected %d'
                          % (len(raw), ENCODER_CHANNELS * 4))
        return list(struct.unpack('<4i', bytes(raw)))

# 麦轮子底盘速度处理
class MecanumChassis:
    # A = 110  # mm
    # B = 97.5  # mm
    # WHEEL_DIAMETER = 96.5  # mm
    # PULSE_PER_CYCLE = 44
    def __init__(self, a=110, b=97.5, wheel_diameter=96.5, pulse_per_cycle=44 * 178):
        self.motor_controller = EncoderMotorController(1)
        self.a = a
        self.b = b
        self.wheel_diameter = wheel_diameter
        self.pulse_per_cycle = pulse_per_cycle
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def speed_covert(self, speed):
        """
        covert speed mm/s to pulse/10ms
        :param speed:
        :return:
        """
        return speed / (math.pi * self.wheel_diameter) * self.pulse_per_cycle * 0.01  # pulse/10ms

    def reset_motors(self):
        # set_speed(speed, motor_id=None): pass one 4-channel vector, not
        # (motor_id, speed). The old positional calls set motor_id=0 and
        # failed to clear any wheel output.
        self.motor_controller.set_speed([0, 0, 0, 0])
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def set_velocity(self, velocity, direction, angular_rate, fake=False):
        """
        Use polar coordinates to control moving
        motor3 v2|  ↑  |v1 motor1
                 |     |
        motor4 v3|     |v4 motor2
        :param velocity: mm/s
        :param direction: Moving direction 0~360deg, 180deg<--- ↑ ---> 0deg
        :param angular_rate:  The speed at which the chassis rotates
        :param fake:
        :return:
        """
        velocity = -velocity
        angular_rate = -angular_rate
        
        rad_per_deg = math.pi / 180
        vx = velocity * math.cos(direction * rad_per_deg)
        vy = velocity * math.sin(direction * rad_per_deg)
        vp = angular_rate * (self.a + self.b)
        v1 = vy - vx + vp
        v2 = vy + vx - vp
        v3 = vy - vx - vp
        v4 = vy + vx + vp
        v_s = [int(self.speed_covert(v)) for v in [-v1, v4, v2, -v3]]
        if fake:
            return v_s

        self.motor_controller.set_speed(v_s)
        self.velocity = velocity
        self.direction = direction
        self.angular_rate = angular_rate
    
    # 控制XY方向平移函数
    def translation(self, velocity_x, velocity_y, fake=False):
        global slow_en
        
        velocity = math.sqrt(velocity_x ** 2 + velocity_y ** 2)
        if velocity_x == 0:
            direction = 90 if velocity_y >= 0 else 270  # pi/2 90deg, (pi * 3) / 2  270deg
        else:
            if velocity_y == 0:
                direction = 0 if velocity_x > 0 else 180
            else:
                direction = math.atan(velocity_y / velocity_x)  # θ=arctan(y/x) (x!=0)
                direction = direction * 180 / math.pi
                if velocity_x < 0:
                    direction += 180
                else:
                    if velocity_y < 0:
                        direction += 360
        if fake:
            return velocity, direction
        
        else:
            _start_motion(velocity, direction, 0)


last_velocity = 0
last_angular = 0
last_direction = 90
# 缓变速处理函数
def _start_motion(velocity, direction, angular):
    """Start one cancellable motion command and supersede older commands."""
    global motion_generation, th
    with motion_lock:
        motion_generation += 1
        generation = motion_generation
        worker = Thread(target=slow_velocity,
                        args=(velocity, direction, angular, generation))
        worker.setDaemon(True)
        th = worker
    worker.start()


def slow_velocity(velocity, direction, angular, generation=None):
    global th
    global last_velocity
    global last_angular
    global last_direction

    with motion_lock:
        if generation is None:
            generation = motion_generation
        if generation != motion_generation or rospy.is_shutdown():
            return
        current_velocity = last_velocity
        current_angular = last_angular
        current_direction = last_direction

    added_v = 30
    added_a = 0.2
    diff_velocity = velocity - current_velocity
    diff_angular = angular - current_angular

    if abs(diff_velocity) >= added_v or abs(diff_angular) >= added_a:
        if diff_velocity > added_v:
            dv = added_v
        elif diff_velocity < -added_v:
            dv = -added_v
        else:
            dv = 0

        if diff_angular > added_a:
            da = added_a
        elif diff_angular < -added_a:
            da = -added_a
        else:
            da = 0

        direction_ = direction
        if velocity == 0 and direction <= 0 and angular == 0:
            direction = current_direction

        while abs(diff_velocity) > added_v or abs(diff_angular) > added_a:
            with motion_lock:
                if generation != motion_generation or rospy.is_shutdown():
                    return
                if abs(diff_velocity) >= added_v:
                    current_velocity += dv
                if abs(diff_angular) >= added_a:
                    current_angular += da
                diff_velocity = velocity - current_velocity
                diff_angular = angular - current_angular
                current_velocity = round(current_velocity, 2)
                current_angular = round(current_angular, 2)
                last_velocity = current_velocity
                last_angular = current_angular
                last_direction = direction_
                chassis.set_velocity(current_velocity, direction,
                                     current_angular)
            rospy.sleep(0.05)

        with motion_lock:
            if generation != motion_generation or rospy.is_shutdown():
                return
            last_velocity = velocity
            last_angular = angular
            last_direction = direction_
            chassis.set_velocity(last_velocity, direction, last_angular)

    else:
        with motion_lock:
            if generation != motion_generation or rospy.is_shutdown():
                return
            last_angular = angular
            last_velocity = velocity
            last_direction = direction
            chassis.set_velocity(velocity, direction, angular)

    with motion_lock:
        if generation == motion_generation and th is current_thread():
            th = None

# 平移控制回调函数
def Set_Translation(msg):
    velocity_x = msg.velocity_x
    velocity_y = msg.velocity_y
    chassis.translation(velocity_x, velocity_y)

# 普通控制回调函数
def Set_Velocity(msg):
    velocity = msg.velocity
    direction = msg.direction
    angular = round(msg.angular,2)
    _start_motion(velocity, direction, angular)


def Immediate_Stop(_msg):
    """切断所有缓变速线程，并直接把四个电机寄存器清零。"""
    global motion_generation, th
    global last_velocity, last_angular, last_direction
    global wall_follow_active

    # 与每次实际 I2C 输出共用同一把锁，保证清零后旧线程不能再补写
    # 一帧非零速度。把 generation 递增后，所有旧的缓变速线程都会退出。
    with motion_lock:
        motion_generation += 1
        th = None
        last_velocity = 0
        last_angular = 0
        last_direction = 90
        wall_follow_active = False
        chassis.reset_motors()
    rospy.logwarn('收到底盘立即停止命令，四路电机输出已清零')


def Wall_Follow_Status(msg):
    """Track the autonomous wall-follow lease without affecting manual drive."""
    global wall_follow_active, wall_follow_last_status
    try:
        state = json.loads(msg.data or '').get('state')
    except (TypeError, ValueError):
        rospy.logwarn_throttle(5.0, '忽略格式错误的 /wall_follow/status')
        return
    wall_follow_active = state in WALL_FOLLOW_ACTIVE_STATES
    wall_follow_last_status = time.monotonic()


def Wall_Follow_Watchdog(_event):
    """Stop the motors if wall_follow dies while it still owns motion."""
    global wall_follow_active
    if not wall_follow_active:
        return
    age = time.monotonic() - wall_follow_last_status
    if age > wall_follow_watchdog_timeout:
        chassis.reset_motors()
        wall_follow_active = False
        rospy.logwarn('wall_follow 心跳超时 %.2fs，已切断电机输出', age)


def Publish_Encoder(_event):
    """Publish cumulative wheel counts for odom_node.py."""
    try:
        encoder_pub.publish(Int32MultiArray(
            data=chassis.motor_controller.read_encoder_counts()))
    except Exception as e:
        rospy.logwarn_throttle(5.0, '读取底盘编码器失败: %s', e)


if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('chassis_control', log_level=rospy.DEBUG)
    # app通信服务
    set_velocity_sub = rospy.Subscriber('/chassis_control/set_velocity', SetVelocity, Set_Velocity)
    set_translation_sub = rospy.Subscriber('/chassis_control/set_translation', SetTranslation, Set_Translation)
    wall_follow_status_sub = rospy.Subscriber('/wall_follow/status', String,
                                               Wall_Follow_Status, queue_size=1)

    chassis = MecanumChassis()
    # 放在底盘对象初始化之后，避免收到残留消息时访问未初始化的 chassis。
    immediate_stop_sub = rospy.Subscriber('/chassis_control/stop', Empty,
                                          Immediate_Stop, queue_size=1)
    wall_follow_watchdog_timeout = rospy.get_param(
        '~wall_follow_watchdog_timeout', DEFAULT_WALL_FOLLOW_WATCHDOG)
    if wall_follow_watchdog_timeout <= 0:
        rospy.logwarn('~wall_follow_watchdog_timeout=%s 无效，回退到 %.1f 秒',
                      wall_follow_watchdog_timeout, DEFAULT_WALL_FOLLOW_WATCHDOG)
        wall_follow_watchdog_timeout = DEFAULT_WALL_FOLLOW_WATCHDOG
    wall_follow_watchdog_timer = rospy.Timer(
        rospy.Duration(0.2), Wall_Follow_Watchdog)
    encoder_pub = rospy.Publisher(
        '/chassis_control/encoder_counts', Int32MultiArray, queue_size=10)
    encoder_rate = rospy.get_param('~encoder_rate', DEFAULT_ENCODER_RATE)
    if encoder_rate <= 0:
        rospy.logwarn('~encoder_rate=%s 无效，回退到 %.1f Hz',
                      encoder_rate, DEFAULT_ENCODER_RATE)
        encoder_rate = DEFAULT_ENCODER_RATE
    encoder_timer = rospy.Timer(
        rospy.Duration(1.0 / encoder_rate), Publish_Encoder)
    rospy.loginfo('publishing /chassis_control/encoder_counts at %.1f Hz',
                  encoder_rate)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
