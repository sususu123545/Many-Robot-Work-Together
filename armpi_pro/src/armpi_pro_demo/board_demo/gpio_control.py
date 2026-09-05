import gpiod
import time
import signal
import sys
print('''
**********************************************************
********功能:幻尔科技树莓派扩展板，GPIO应用例程*********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')
chip = gpiod.Chip("gpiochip4")
fanPin1 = chip.get_line(8)
fanPin2 = chip.get_line(7)

fanPin1.request(consumer="pin1", type=gpiod.LINE_REQ_DIR_OUT)
fanPin2.request(consumer="pin2", type=gpiod.LINE_REQ_DIR_OUT)

def set_direction(pin1_value, pin2_value):
    fanPin1.set_value(pin1_value)
    fanPin2.set_value(pin2_value)

def rotate_clockwise():
    set_direction(1, 0)

def rotate_counterclockwise():
    set_direction(0, 1)

def stop_rotation():
    set_direction(0, 0)

def cleanup(signal, frame):
    stop_rotation()
    fanPin1.release()
    fanPin2.release()
    chip.close()
    print("已关闭")
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)

try:
    rotate_clockwise()
    time.sleep(3)

    rotate_counterclockwise()
    time.sleep(3)

    stop_rotation()

except KeyboardInterrupt:
    cleanup(None, None)