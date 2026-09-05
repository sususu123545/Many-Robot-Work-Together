#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import rospy
import sqlite3 as sql
from bus_servo_control import BusServoControl
from ros_robot_controller.msg import SetBusServoState, BusServoState

class ActionGroupController():
    runningAction = False
    stopRunning = False

    action_path = os.path.split(os.path.realpath(__file__))[0]

    def __init__(self, board=None, use_ros=False):
        if use_ros:
            while not rospy.is_shutdown():
                try:
                    if rospy.get_param('/ros_robot_controller/init_finish'):
                        break
                except:
                    rospy.sleep(0.1)
            rospy.sleep(1)
            self.servo_state_pub = rospy.Publisher('ros_robot_controller/bus_servo/set_state', SetBusServoState, queue_size=10)
        else:
            self.board = BusServoControl(board)
        self.use_ros = use_ros

    def stop_servo(self):
        if self.use_ros:
            data = []
            msg = SetBusServoState()
            for i in range(24):
                servo_state = BusServoState()
                servo_state.present_id = [1, i + 1]
                servo_state.stop = [1, 1]
                data.append(servo_state)
            msg.state = data
            self.servo_state_pub.publish(msg)
        else:
            self.board.stopBusServo([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]) 

    def stop_action_group(self):
        self.stopRunning = True

    def runAction(self, actNum):
        '''
        运行动作组，无法发送stop停止信号
        :param actNum: 动作组名字 ， 字符串类型
        :param times:  运行次数
        :return:
        '''
        if actNum is None:
            return
        actNum = os.path.join(self.action_path, 'ActionGroups', actNum + ".d6a")
        self.stopRunning = False
        if os.path.exists(actNum) is True:
            if self.runningAction is False:
                self.runningAction = True
                ag = sql.connect(actNum)
                cu = ag.cursor()
                cu.execute("select * from ActionGroup")
                while True:
                    act = cu.fetchone()
                    if self.stopRunning is True:
                        self.stopRunning = False                   
                        break
                    if act is not None:
                        for i in range(0, len(act)-2, 1):
                            if self.use_ros:
                                data = SetBusServoState()
                                servo_state = BusServoState()
                                servo_state.present_id = [1, i + 1]
                                servo_state.position = [1, act[2 + i]]
                                data.state = [servo_state]
                                data.duration = act[1]/1000.0
                                self.servo_state_pub.publish(data)
                            else:
                                self.board.setBusServoPulse(i+1, act[2 + i], act[1])
                        time.sleep(float(act[1])/1000.0)
                    else:   # 运行完才退出
                        break
                self.runningAction = False
                
                cu.close()
                ag.close()
        else:
            self.runningAction = False
            print("未能找到动作组文件")
