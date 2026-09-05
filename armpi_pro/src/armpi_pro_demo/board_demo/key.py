#!/usr/bin/env python3
# encoding:utf-8
import time
import gpiod
import ros_robot_controller_sdk as rrc
import signal
print('''
**********************************************************
********功能:幻尔科技树莓派扩展板，按键控制RGB灯**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')
board = rrc.Board()
start = True

def handle_sigint(signal, frame):
    global start
    start = False
    board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])  # 设置 RGB 为黑色，关闭所有灯

# 注册信号处理程序，捕获 Ctrl+C 信号
signal.signal(signal.SIGINT, handle_sigint)

try:
    key1_pin = 13
    chip = gpiod.Chip("gpiochip4")
    key1 = chip.get_line(key1_pin)
    key1.request(consumer="key1", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

    key2_pin = 23
    key2 = chip.get_line(key2_pin)
    key2.request(consumer="key2", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

    while start:
        # 检查按键状态
        key1_state = key1.get_value()
        key2_state = key2.get_value()

        if key1_state == 0:
            # 按键1被按下，设置 RGB 为红色
            board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])

        if key2_state == 0:
            # 按键2被按下，设置 RGB 为蓝色
            board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])

        print('\rkey1: {} key2: {}'.format(key1_state, key2_state), end='', flush=True)  # 打印 key 状态
        time.sleep(0.001)

except Exception as e:
    print('发生异常:', str(e))
    print('按键默认被 hw_button_scan 占用，需要先关闭服务')
    print('sudo systemctl stop hw_button_scan.service')

finally:
    if chip:
        chip.close()