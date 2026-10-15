"""CH341-safe transport adapted from car_setup/bmi270_imu_node.py.

The existing third-party bmi270_py configuration blob is an external runtime
dependency, not imported from a ROS 1 node or silently installed from PyPI.
"""
import glob
import importlib
import math
import os
import sys
import time


def find_ch341_bus():
    for path in sorted(glob.glob('/sys/class/i2c-adapter/i2c-*/name')):
        with open(path, encoding='utf-8') as handle:
            name = handle.read().strip().lower()
        bus = int(os.path.basename(os.path.dirname(path)).split('-')[-1])
        if 'i2c-ch341-usb' in name and os.path.exists(f'/dev/i2c-{bus}'):
            return bus
    raise RuntimeError('CH341 I2C adapter missing; host kernel driver is required')


def make_sensor(bus, library_path):
    if not os.path.isfile(os.path.join(library_path, 'bmi270', 'config_file.py')):
        raise RuntimeError(f'Missing verified BMI270 library at {library_path}')
    if library_path not in sys.path:
        sys.path.insert(0, library_path)
    module = importlib.import_module('bmi270.BMI270')
    registers = importlib.import_module('bmi270.registers')
    definitions = importlib.import_module('bmi270.definitions')
    config = importlib.import_module('bmi270.config_file').bmi270_config_file
    module.I2C_BUS = bus
    sensor = None
    for address in (0x68, 0x69):
        try:
            candidate = module.BMI270(address)
            if candidate.read_register(registers.CHIP_ID_ADDRESS) == 0x24:
                sensor = candidate
                break
            candidate.bus.close()
        except OSError:
            if 'candidate' in locals():
                candidate.bus.close()
    if sensor is None:
        raise RuntimeError('BMI270 chip ID 0x24 not found at 0x68/0x69')
    try:
        sensor.write_register(registers.CMD, 0xB6)
        time.sleep(0.005)
        sensor.write_register(registers.PWR_CONF, 0x00)
        time.sleep(0.001)
        sensor.write_register(registers.INIT_CTRL, 0x00)
        for offset in range(0, len(config), 24):
            words = offset // 2
            sensor.write_register(registers.INIT_ADDR_0, words & 0x0f)
            sensor.write_register(registers.INIT_ADDR_1, (words >> 4) & 0xff)
            sensor.bus.write_i2c_block_data(sensor.address, registers.INIT_DATA, config[offset:offset + 24])
            time.sleep(0.00002)
        sensor.write_register(registers.INIT_CTRL, 0x01)
        time.sleep(0.025)
        if sensor.read_register(registers.INTERNAL_STATUS) != 1:
            raise RuntimeError('BMI270 firmware initialization failed')
        sensor.set_mode('performance')
        sensor.set_acc_range(definitions.ACC_RANGE_2G)
        sensor.set_gyr_range(definitions.GYR_RANGE_1000)
        return sensor
    except Exception:
        sensor.bus.close()
        raise


def read_sample(sensor):
    raw = sensor.bus.read_i2c_block_data(sensor.address, 0x0c, 12)
    if len(raw) != 12:
        raise OSError('Incomplete BMI270 accel+gyro transfer')
    def signed16(lo, hi):
        value = lo | hi << 8
        return value - 65536 if value & 0x8000 else value
    accel = [signed16(raw[i], raw[i + 1]) * (2 * 9.80665 / 32768) for i in (0, 2, 4)]
    gyro = [signed16(raw[i], raw[i + 1]) * math.radians(1000 / 32768) for i in (6, 8, 10)]
    return accel, gyro
