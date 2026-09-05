#!/usr/bin/python3
# coding=utf8
# Date:2024/04/01

# 第8章 ArmPi Pro标准课程\第1课 视觉巡线

import sys
import cv2
import time
import math
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image

# 自定义消息类型
from chassis_control.msg import *
from visual_patrol.srv import SetTarget
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from ros_robot_controller.msg import RGBState, RGBsState, BuzzerState

# 底层控制库
from armpi_pro import pid
from armpi_pro import bus_servo_control
from kinematics import ik_transform


lock = RLock()
ik = ik_transform.ArmIK()

move = False
target_en = False
target_color = 'None'
__isRunning = False

img_w = 640
x_pid = pid.PID(P=0.003, I=0.0001, D=0.0001)  # pid初始化

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'yellow': (0, 255, 255),
    'white': (255, 255, 255),
}

def set_rgb(r, g, b):
    rgb1 = RGBState()
    rgb1.id = 1
    rgb1.r = r
    rgb1.g = g
    rgb1.b = b 
    rgb2 = RGBState()
    rgb2.id = 2 
    rgb2.r = r 
    rgb2.g = g 
    rgb2.b = b 
    msg = RGBsState()
    msg.data = [rgb1, rgb2]
    rgb_pub.publish(msg)


# 初始位置
def initMove(delay=True):
    with lock:
        bus_servo_control.set_servos(joints_pub, 1.5, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    if delay:
        rospy.sleep(2)

# 变量重置
def reset():
    global move
    global target_en
    global target_color
    
    with lock:
        move = False
        target_en = False
        x_pid.clear()
        target_color = 'None'

# app初始化调用
def init():
    rospy.loginfo("visual patrol Init")
    initMove()
    reset()

# 图像处理结果回调函数
def run(msg):
    global move
    
    center_x = msg.center_x
    center_y = msg.center_y
    width = msg.data
    if __isRunning:
        
        if width > 0:
            # PID算法巡线
            if abs(center_x - img_w/2) < 20: # 目标横坐标与画面中心坐标的差值小于20像素点，机器人不做处理
                center_x = img_w/2
            x_pid.SetPoint = img_w/2      # 设定
            x_pid.update(center_x)        # 当前
            dx = round(x_pid.output, 2)   # 输出
            dx = 0.8 if dx > 0.8 else dx
            dx = -0.8 if dx < -0.8 else dx
            set_velocity.publish(100, 90, dx)
            move = True
                
        else:
            if move: # 判断之前是否有移动，避免重复发送停止指令
                move = False
                set_velocity.publish(0, 90, 0)
        

result_sub = None
heartbeat_timer = None
# APP enter服务回调函数
def enter_func(msg):
    global lock
    global result_sub
    
    rospy.loginfo("enter visual patrol")
    init()
    with lock:
        if result_sub is None:
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            result_sub = rospy.Subscriber('/visual_processing/result', Result, run)
            
    return [True, 'enter']

# APP exit服务回调函数
def exit_func(msg):
    global lock
    global result_sub
    global __isRunning
    global heartbeat_timer
    
    rospy.loginfo("exit visual patrol")
    with lock:
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        __isRunning = False
        reset()
        set_velocity.publish(0, 90, 0)
        try:
            if result_sub is not None:
                result_sub.unregister()
                result_sub = None
            if heartbeat_timer is not None:
                heartbeat_timer.cancel()
                heartbeat_timer = None
        except BaseException as e:
            rospy.loginfo('%s', e)
        
    return [True, 'exit']

# 开始运行函数
def start_running():
    global lock
    global __isRunning
    
    rospy.loginfo("start running visual patrol")
    with lock:
        __isRunning = True

# 停止运行函数
def stop_running():
    global lock
    global __isRunning
    
    rospy.loginfo("stop running visual patrol")
    with lock:
        reset()
        __isRunning = False
        initMove(delay=False)
        set_rgb(0, 0, 0)
        set_velocity.publish(0,90,0)
        rospy.ServiceProxy('/visual_processing/set_running', SetParam)()

# APP set_running服务回调函数
def set_running(msg):
    
    if msg.data:
        start_running()
    else:
        stop_running()
        
    return [True, 'set_running']

# APP set_target服务回调函数
def set_target(msg):
    global lock
    global target_en
    global target_color
    
    rospy.loginfo("%s", msg)
    with lock:
        target_color = msg.data
        #set_rgb(0,0,255)
        rospy.loginfo(range_rgb[target_color][0])
        set_rgb(range_rgb[target_color][2], range_rgb[target_color][1], range_rgb[target_color][0])
        visual_running = rospy.ServiceProxy('/visual_processing/set_running', SetParam)
        visual_running('line',target_color)
        rospy.sleep(0.1)
        target_en = True
        
    return [True, 'set_target']

# APP heartbeat服务回调函数
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/visual_patrol/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('visual_patrol', log_level=rospy.DEBUG)
    # 舵机发布
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # app通信服务
    enter_srv = rospy.Service('/visual_patrol/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/visual_patrol/exit', Trigger, exit_func)
    running_srv = rospy.Service('/visual_patrol/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/visual_patrol/set_target', SetTarget, set_target)
    heartbeat_srv = rospy.Service('/visual_patrol/heartbeat', SetBool, heartbeat_srv_cb)
    # 麦轮底盘控制
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    set_translation = rospy.Publisher('/chassis_control/set_translation', SetTranslation, queue_size=1)
    # 蜂鸣器
    buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
    # rgb 灯
    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    rospy.sleep(0.5) # pub之后必须延时才能生效
    
    initMove()
    msg = BuzzerState()
    msg.freq = 1900
    msg.on_time = 0.1
    msg.off_time = 0.9
    msg.repeat = 1
    buzzer_pub.publish(msg)

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        
        msg = SetTarget()
        msg.data = 'red'
        
        set_target(msg)
        start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
