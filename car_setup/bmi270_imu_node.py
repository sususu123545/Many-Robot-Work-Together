#!/usr/bin/env python3
"""Publish the external BMI270 connected through a CH341 I2C adapter.

The CH341 adapter is intentionally discovered by its adapter name instead of
hard-coding a bus number.  This matters because Linux may assign a different
bus number after a reboot or USB re-enumeration.

The BMI270 initialization image is sent in 26-byte pieces.  The CH341 driver
uses a 32-byte USB endpoint and therefore accepts at most 26 I2C payload bytes
per message (the extra bytes are the I2C command/address/stop framing).
"""

import argparse
import glob
import importlib
import math
import os
import sys
import time

import rospy
from sensor_msgs.msg import Imu


CH341_NAME = "i2c-ch341-usb"
BMI270_ADDRESSES = (0x68, 0x69)
BMI270_CHIP_ID = 0x24
GRAVITY = 9.80665
ACC_RANGE_G = 2.0
GYR_RANGE_DPS = 1000.0

# Static recalibration measured on 2026-09-23 from 499 samples while the
# vehicle was stationary; encoder delta was zero.  The values are configurable
# through ROS params so a later calibration does not require changing the node.
DEFAULT_GYRO_BIAS_RAD_S = (
    0.002870237,
    0.002160416,
    -0.000748247,
)


def find_ch341_bus():
    """Return the Linux I2C bus number belonging to the CH341 adapter."""
    for name_path in sorted(glob.glob("/sys/class/i2c-adapter/i2c-*/name")):
        try:
            with open(name_path, "r", encoding="utf-8") as handle:
                name = handle.read().strip().lower()
        except OSError:
            continue
        if CH341_NAME in name:
            bus = int(os.path.basename(os.path.dirname(name_path)).split("-")[-1])
            if os.path.exists("/dev/i2c-%d" % bus):
                return bus
    return None


def load_config_for_ch341(sensor, config, registers):
    """Load the BMI270 firmware using CH341-safe I2C block sizes."""
    if sensor.read_register(registers.INTERNAL_STATUS) == 0x01:
        return

    sensor.write_register(registers.PWR_CONF, 0x00)
    time.sleep(0.001)
    sensor.write_register(registers.INIT_CTRL, 0x00)

    # BMI270 advances INIT_ADDR by burst_length / 2.  Use 24-byte bursts:
    # 24 data bytes + the SMBus register byte stay within CH341's limit, and
    # 24 is even as required by the BMI270 initialization address.
    for offset in range(0, len(config), 24):
        address_words = offset // 2
        sensor.write_register(registers.INIT_ADDR_0, address_words & 0x0F)
        sensor.write_register(registers.INIT_ADDR_1, (address_words >> 4) & 0xFF)
        sensor.bus.write_i2c_block_data(
            sensor.address,
            registers.INIT_DATA,
            config[offset : offset + 24],
        )
        time.sleep(0.00002)

    sensor.write_register(registers.INIT_CTRL, 0x01)
    time.sleep(0.025)
    status = sensor.read_register(registers.INTERNAL_STATUS)
    if status != 0x01:
        raise RuntimeError("BMI270 init failed: INTERNAL_STATUS=0x%02x" % status)


def make_sensor(bus):
    """Create and initialize the BMI270 package object on a dynamic bus."""
    sys.path.insert(0, "/home/ubuntu/share/tmp/bmi270_py")
    bmi_module = importlib.import_module("bmi270.BMI270")
    config_module = importlib.import_module("bmi270.config_file")
    registers = importlib.import_module("bmi270.registers")
    definitions = importlib.import_module("bmi270.definitions")

    # The third-party class uses a module-level I2C_BUS constant.
    bmi_module.I2C_BUS = bus
    sensor = None
    chip_id = None
    for address in BMI270_ADDRESSES:
        candidate = bmi_module.BMI270(address)
        candidate_id = candidate.read_register(registers.CHIP_ID_ADDRESS)
        if candidate_id == BMI270_CHIP_ID:
            sensor = candidate
            chip_id = candidate_id
            break
    if sensor is None:
        raise RuntimeError(
            "BMI270 not found at 0x68/0x69; last CHIP_ID=0x%02x"
            % (chip_id if chip_id is not None else 0)
        )

    # Start from a clean BMI270 state after a failed/partial USB transfer.
    sensor.write_register(registers.CMD, 0xB6)
    time.sleep(0.005)
    load_config_for_ch341(sensor, config_module.bmi270_config_file, registers)
    sensor.set_mode("performance")
    sensor.set_acc_range(definitions.ACC_RANGE_2G)
    sensor.set_gyr_range(definitions.GYR_RANGE_1000)
    return sensor


def read_sample(sensor):
    """Read accel+gyro in one 12-byte transaction and convert to SI units."""
    raw = sensor.bus.read_i2c_block_data(sensor.address, 0x0C, 12)

    def signed16(lo, hi):
        value = lo | (hi << 8)
        return value - 65536 if value & 0x8000 else value

    accel = [signed16(raw[i], raw[i + 1]) for i in (0, 2, 4)]
    gyro = [signed16(raw[i], raw[i + 1]) for i in (6, 8, 10)]
    acc_scale = ACC_RANGE_G * GRAVITY / 32768.0
    gyr_scale = GYR_RANGE_DPS * math.pi / 180.0 / 32768.0
    return (
        [value * acc_scale for value in accel],
        [value * gyr_scale for value in gyro],
    )


def run(test_once=False):
    bus = find_ch341_bus()
    if bus is None:
        raise RuntimeError("CH341 I2C adapter not found")

    sensor = make_sensor(bus)
    accel, gyro = read_sample(sensor)
    if test_once:
        print("BMI270 SAMPLE accel=%s gyro=%s" % (accel, gyro))
    rospy.loginfo("BMI270 ready on /dev/i2c-%d @ 0x%02x", bus, sensor.address)
    rospy.loginfo("BMI270 sample accel=%s gyro=%s", accel, gyro)
    if test_once:
        return

    frame_id = rospy.get_param("~frame_id", "imu_link")
    topic = rospy.get_param("~topic", "/bmi270/imu_raw")
    compat_topic = rospy.get_param(
        "~compat_topic", "/ros_robot_controller/imu_raw"
    )
    rate_hz = float(rospy.get_param("~rate", 50.0))
    gyro_bias = tuple(
        float(rospy.get_param("~gyro_bias_%s" % axis, default))
        for axis, default in zip("xyz", DEFAULT_GYRO_BIAS_RAD_S)
    )
    rospy.loginfo(
        "BMI270 gyro bias correction enabled: x=%.9f y=%.9f z=%.9f rad/s",
        *gyro_bias,
    )
    pub = rospy.Publisher(topic, Imu, queue_size=10)
    compat_pub = rospy.Publisher(compat_topic, Imu, queue_size=10)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        try:
            accel, gyro = read_sample(sensor)
            corrected_gyro = [gyro[i] - gyro_bias[i] for i in range(3)]
            message = Imu()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = frame_id
            message.orientation_covariance[0] = -1.0
            message.linear_acceleration.x = accel[0]
            message.linear_acceleration.y = accel[1]
            message.linear_acceleration.z = accel[2]
            message.angular_velocity.x = corrected_gyro[0]
            message.angular_velocity.y = corrected_gyro[1]
            message.angular_velocity.z = corrected_gyro[2]
            message.linear_acceleration_covariance = [
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0,
                0.0, 0.0, 0.04,
            ]
            message.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]
            pub.publish(message)
            if compat_topic != topic:
                compat_pub.publish(message)
        except Exception as error:  # keep ROS alive for USB reconnects
            rospy.logwarn_throttle(5.0, "BMI270 read failed: %s", error)
        rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test-once", action="store_true")
    args, _ = parser.parse_known_args()
    if not args.test_once:
        rospy.init_node("bmi270_imu", anonymous=False)
    try:
        run(test_once=args.test_once)
    except Exception as error:
        if args.test_once:
            print("BMI270 ERROR: %s" % error, file=sys.stderr)
            raise
        rospy.logfatal("BMI270 startup failed: %s", error)
        raise
