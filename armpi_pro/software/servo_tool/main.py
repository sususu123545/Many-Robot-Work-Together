#!/usr/bin/env python3
# encoding: utf-8
import os
import re
import cv2
import sys
import copy
import math
import time
import threading
from socket import * 
from ui import Ui_Form
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from servo_controller import *
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtSql import QSqlDatabase, QSqlQuery

class MainWindow(QtWidgets.QWidget, Ui_Form):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.set_window_position()
        
        self.path = os.path.split(os.path.realpath(__file__))[0]
        self.setWindowIcon(QIcon(os.path.join(self.path, 'resources/app.png')))
        self.message = QMessageBox()

        self.radioButton_zn.toggled.connect(lambda: self.language(self.radioButton_zn))
        self.radioButton_en.toggled.connect(lambda: self.language(self.radioButton_en))        
        self.chinese = True
        try:
            if os.environ['ASR_LANGUAGE'] == 'Chinese':
                self.radioButton_zn.setChecked(True)
            else:
                self.chinese = False
                self.radioButton_en.setChecked(True)
        except:
            self.radioButton_zn.setChecked(True)
        #################################界面#######################################
        self.id = 0
        self.dev = 0
        self.servoTemp = 0
        self.servoMin = 0
        self.servoMax = 0
        self.servoMinV = 0
        self.servoMaxV = 0
        self.servoMove = 0
        self.lineEdit_input = False
        self.readOrNot = False
        self.haved_read = False
        self.start_time = 0
        self.plus_pressed = False
        self.reduce_pressed = False

        self.horizontalSlider_servoTemp.valueChanged.connect(lambda: self.horizontalSlider_valuechange('servoTemp'))
        self.horizontalSlider_servoMin.valueChanged.connect(lambda: self.horizontalSlider_valuechange('servoMin'))
        self.horizontalSlider_servoMax.valueChanged.connect(lambda: self.horizontalSlider_valuechange('servoMax'))
        self.horizontalSlider_servoMinV.valueChanged.connect(lambda: self.horizontalSlider_valuechange('servoMinV'))
        self.horizontalSlider_servoMaxV.valueChanged.connect(lambda: self.horizontalSlider_valuechange('servoMaxV'))
        self.horizontalSlider_servoMove.valueChanged.connect(lambda: self.horizontalSlider_valuechange('servoMove'))
        
        self.horizontalSlider1.valueChanged.connect(lambda: self.horizontalSlider_valuechange('servoID'))
        self.horizontalSlider2.valueChanged.connect(lambda: self.horizontalSlider_valuechange('servoDev'))
        
        self.pushButton_read.pressed.connect(lambda: self.pushbutton_clicked('read'))
        self.pushButton_set.pressed.connect(lambda: self.pushbutton_clicked('set'))
        self.pushButton_default.pressed.connect(lambda: self.pushbutton_clicked('default'))
        self.pushButton_quit2.pressed.connect(lambda: self.pushbutton_clicked('quit2'))
        self.pushButton_resetPos.pressed.connect(lambda: self.pushbutton_clicked('resetPos'))
        self.lineEdit1.setValidator(QIntValidator(-125, 125))
        self.lineEdit2.setValidator(QIntValidator(0, 253))
        self.lineEdit1.textEdited[str].connect(lambda: self.lineEdit_valuechange('servoID'))
        self.lineEdit2.textEdited[str].connect(lambda: self.lineEdit_valuechange('servoDev'))
       
        # +
        self.plus_object = []
        for i in range(2):
            object_name = self.findChild(QPushButton, 'pushButton_plus' + str(i + 1))
            object_name.pressed.connect(self.button_clicked)
            object_name.released.connect(self.button_released)
            self.plus_object.append(object_name)
        
        # -
        self.reduce_object = []
        for i in range(2):
            object_name = self.findChild(QPushButton, 'pushButton_reduce' + str(i + 1))
            object_name.pressed.connect(self.button_clicked)
            object_name.released.connect(self.button_released)
            self.reduce_object.append(object_name)

        self.slider_object = []
        for i in range(2):
            object_name = self.findChild(QSlider, 'horizontalSlider' + str(i + 1))
            self.slider_object.append(object_name)

        self.lineEdit_object = []
        for i in range(2):
            object_name = self.findChild(QLineEdit, 'lineEdit' + str(i + 1))
            self.lineEdit_object.append(object_name)
        enable_reception(False)
        # 长按检测
        self.button_timer = QTimer()
        self.button_timer.timeout.connect(self.button_pressed_check)
        self.button_timer.start(20)

    def set_window_position(self):
        # 窗口居中
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def language(self, name):
        if self.radioButton_zn.isChecked() and name.text() == '中文':
            self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), "舵机调试工具")
            self.chinese = True
            self.label_id.setText("舵机ID")
            self.label_dev.setText("舵机偏差")
            self.label_temp.setText("温度范围")
            self.label_angle.setText("角度范围")
            self.label_vin.setText("电压范围")
            self.label_current_id.setText("当前舵机ID: ")
            self.label_current_angle.setText("当前角度: ")
            self.label_current_temp.setText("当前温度: ")
            self.label_current_vin.setText("当前电压: ")
            self.label_reset.setText("舵机调试")
            self.pushButton_resetPos.setText("中位")
            self.pushButton_quit2.setText("退出")
            self.label_tips.setText("<html><head/><body><p><span style=\" color:#ff0000;\">注意：使用下面的功能时，请确保控制器只连接了一个舵机，否则会引起冲突!</span></p></body></html>")
            self.pushButton_read.setText("读取")
            self.pushButton_set.setText("设置")
            self.pushButton_default.setText("默认")
            self.message_From('使用此面板时，请确保只连接了一个舵机，否则会引起冲突！')
        elif self.radioButton_en.isChecked() and name.text() == 'English':
            self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), "Servo Tool")
            self.chinese = False
            self.label_id.setText("Servo ID")
            self.label_dev.setText("Servo deviation")
            self.label_temp.setText("Temperature")
            self.label_angle.setText("Angle")
            self.label_vin.setText("Voltage")
            self.label_current_id.setText("Current ID: ")
            self.label_current_angle.setText("Current angle: ")
            self.label_current_temp.setText("Current temperature: ")
            self.label_current_vin.setText("Current voltage: ")
            self.label_reset.setText("Servo Test")
            self.pushButton_resetPos.setText("Middle position")
            self.pushButton_quit2.setText("Quit")
            self.label_tips.setText("<html><head/><body><p><span style=\" color:#ff0000;\">Note:before debugging servo,make sure that the servo controller is connected with ONE servo.Otherwise it may cause a conflict!</span></p></body></html>")
            self.pushButton_read.setText("Read")
            self.pushButton_set.setText("Setting")
            self.pushButton_default.setText("Default")
            self.message_From('Before debugging servo,make sure that the servo controller is connected with ONE servo.Otherwise it may cause a conflict!')

    # 弹窗提示函数
    def message_from(self, str):
        try:
            QMessageBox.about(self, '', str)
            time.sleep(0.01)
        except:
            pass
    
    def message_From(self, str):
        self.message_from(str)
   
    # 弹窗提示函数
    def message_delect(self, str):
        messageBox = QMessageBox()
        messageBox.setWindowTitle(' ')
        messageBox.setText(str)
        messageBox.addButton(QPushButton('OK'), QMessageBox.YesRole)
        messageBox.addButton(QPushButton('Cancel'), QMessageBox.NoRole)
        return messageBox.exec_()

    # 窗口退出
    def closeEvent(self, e):        
        result = QMessageBox.question(self,
                                    "关闭窗口提醒",
                                    "exit?",
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.No)
        if result == QMessageBox.Yes:
            enable_reception(True)
            QWidget.closeEvent(self, e)
        else:
            e.ignore()
   
    def lineEdit_valuechange(self, name):
        if name == 'servoID':
            self.lineEdit_input = True
            value = self.lineEdit1.text()
            if value != '':
                self.horizontalSlider1.setValue(int(value))
            self.lineEdit_input = False
        if name == 'servoDev':
            self.lineEdit_input = True
            value = self.lineEdit2.text()
            if value != '':
                self.horizontalSlider2.setValue(int(value))
            self.lineEdit_input = False

    def horizontalSlider_valuechange(self, name):
        if name == 'servoTemp':
            self.temp = str(self.horizontalSlider_servoTemp.value())
            self.label_servoTemp.setText(self.temp + '℃')
        if name == 'servoMin':
            self.servoMin = str(self.horizontalSlider_servoMin.value())
            self.label_servoMin.setText(self.servoMin)
        if name == 'servoMax':
            self.servoMax = str(self.horizontalSlider_servoMax.value())
            self.label_servoMax.setText(self.servoMax)
        if name == 'servoMinV':
            self.servoMinV = str(self.horizontalSlider_servoMinV.value()/10)
            self.label_servoMinV.setText(self.servoMinV + 'V')
        if name == 'servoMaxV':
            self.servoMaxV = str(self.horizontalSlider_servoMaxV.value()/10)
            self.label_servoMaxV.setText(self.servoMaxV + 'V')
        if name == 'servoMove':
            self.servoMove = str(self.horizontalSlider_servoMove.value())            
            self.label_servoMove.setText(self.servoMove)
            setServoPulse(self.id, int(self.servoMove), 0)
        if name == 'servoID':
            if not self.lineEdit_input:
                self.lineEdit1.setText(str(self.horizontalSlider1.value()))
        if name == 'servoDev':
            if not self.lineEdit_input:
                self.lineEdit2.setText(str(self.horizontalSlider2.value()))

    def pushbutton_clicked(self, name):
        if name == 'read':
            try:
                if self.lineEdit1.text() != '-1':
                    self.id = [int(self.lineEdit1.text())]
                else:
                    self.id = getServoID()
                if self.id is None:
                    if self.chinese:
                        self.message_From('读取id失败')
                    else:
                        self.message_From('Failed to read ID')
                    self.lineEdit1.setText('-1')
                    return
                else:
                    self.id = self.id[0]
                self.readOrNot = True
                self.dev = getServoDeviation(self.id)
                if self.dev is None:
                    if self.chinese:
                        self.message_From('读取偏差失败')
                    else:
                        self.message_From('Failed to read deviation')
                    self.dev = 0 
                    self.lineEdit1.setText('-1')
                    return 
                else:
                    self.dev = self.dev[0]
                self.servoTemp = getServoTempLimit(self.id)[0]
                self.servoMin, self.servoMax = getServoAngleLimit(self.id)
                self.servoMinV, self.servoMaxV = getServoVinLimit(self.id)
                self.servoMove = getServoPulse(self.id)[0]
                
                currentVin = getServoVin(self.id)[0]

                currentTemp = getServoTemp(self.id)[0]

                self.lineEdit1.setText(str(self.id))
                self.lineEdit2.setText(str(self.dev))
                self.label_servoCurrentID.setText(str(self.id))

                self.horizontalSlider_servoTemp.setValue(self.servoTemp)
                self.horizontalSlider_servoMin.setValue(self.servoMin)
                self.horizontalSlider_servoMax.setValue(self.servoMax)
                MinV = self.servoMinV
                MaxV = self.servoMaxV            
                self.horizontalSlider_servoMinV.setValue(int(MinV/100))
                self.horizontalSlider_servoMaxV.setValue(int(MaxV/100))

                self.label_servoCurrentP.setText(str(self.servoMove))
                self.label_servoCurrentV.setText(str(round(currentVin/1000.0, 2)) + 'V')
                self.label_servoCurrentTemp.setText(str(currentTemp) + '℃')

                self.horizontalSlider_servoMove.setValue(self.servoMove)
            except BaseException as e:
                print(e)
                self.lineEdit1.setText('-1')
                if self.chinese:
                    self.message_From('读取超时')
                else:
                    self.message_From('Read timeout')
                return
            if self.chinese:
                self.message_From('读取成功')
            else:
                self.message_From('Success')
        if name == 'set':
            if self.readOrNot is False:
                if self.chinese:
                    self.message_From('请先读取，否则无法获取舵机信息，从而进行设置！')
                else:
                    self.message_From('Read first！')
                return
            if self.chinese:
                if self.message_delect('此操作会对所有连接的舵机进行设置，请确保只连接了需要设置的舵机，是否继续？'):
                    return 
            else:
                if self.message_delect('This operation will set all the connected servos, please make sure whether the servo that needs to be set up is connected, whether to continue'):
                    return 
            self.readOrNot = False
            ID = self.lineEdit1.text()
            if ID == '':
                if self.chinese:
                    self.message_From('舵机id参数为空，无法设置')
                else:
                    self.message_From('Please input id')
                return           
            dev = self.lineEdit2.text()
            if dev == '':
                dev = 0
            dev = int(dev)
            if dev > 125 or dev < -125:
                if self.chinese:
                    self.message_From('偏差参数超出可调节范围-125～125，无法设置')
                else:
                    self.message_From('Deviation out of range -125~125')
                return          
            temp = self.horizontalSlider_servoTemp.value()
            pos_min = self.horizontalSlider_servoMin.value()
            pos_max = self.horizontalSlider_servoMax.value()
            if pos_min > pos_max:
                if self.chinese:
                    self.message_From('舵机范围参数错误，无法设置')
                else:
                    self.message_From('Wrong angle range')
                return
            vin_min = self.horizontalSlider_servoMinV.value()
            vin_max = self.horizontalSlider_servoMaxV.value()
            if vin_min > vin_max:
                if self.chinese:
                    self.message_From('舵机电压范围参数错误，无法设置')
                else:
                    self.message_From('Wrong voltage range')
                return
            pos = self.horizontalSlider_servoMove.value()
            
            ID = int(self.lineEdit1.text() )
            try:
                setServoID(self.id, ID)
                time.sleep(0.01)
                self.dev = getServoDeviation(ID)
                if self.dev is None:
                    if self.chinese:
                        self.message_From('id设置失败！')
                    else:
                        self.message_From('Failed！')
                    return
                setServoDeviation(ID, dev)
                time.sleep(0.01)
                saveServoDeviation(ID)
                time.sleep(0.01)
                d = getServoDeviation(ID)
                if d is not None:
                    if d[0] != dev:
                        if self.chinese:
                            self.message_From('偏差设置失败！')
                        else:
                            self.message_From('Failed！')
                        return   
                else:
                    if self.chinese:
                        self.message_From('偏差设置失败！')
                    else:
                        self.message_From('Failed！')
                    return 
                setServoMaxTemp(ID, temp)
                time.sleep(0.01)
                data = getServoTempLimit(ID)
                if data is not None:
                    if data[0] != temp:
                        if self.chinese:
                            self.message_From('温度设置失败！')
                        else:
                            self.message_From('Failed！')
                        return 
                else:
                    if self.chinese:
                        self.message_From('温度设置失败！')
                    else:
                        self.message_From('Failed！')
                    return 
                setServoAngleLimit(ID, pos_min, pos_max)
                time.sleep(0.01)
                data = getServoAngleLimit(ID)
                if data is not None:
                    if data != [pos_min, pos_max]:
                        if self.chinese:
                            self.message_From('角度范围设置失败！')
                        else:
                            self.message_From('Failed！')
                        return
                else:
                    if self.chinese:
                        self.message_From('角度范围设置失败！')
                    else:
                        self.message_From('Failed！')
                    return
                setServoVinLimit(ID, vin_min*100, vin_max*100)
                time.sleep(0.01)
                data = getServoVinLimit(ID)
                if data is not None:
                    if data != [vin_min*100, vin_max*100]:
                        if self.chinese:
                            self.message_From('电压范围设置失败！')
                        else:
                            self.message_From('Failed！')
                        return 
                else:
                    if self.chinese:
                        self.message_From('电压范围设置失败！')
                    else:
                        self.message_From('Failed！')
                    return
                setServoPulse(ID, pos, 0)
            except BaseException as e:
                print(e)
                if self.chinese:
                    self.message_From('设置超时!')
                else:
                    self.message_From('Timeout!')
                return                
            
            self.label_servoCurrentID.setText(str(ID))
            if self.chinese: 
                self.message_From('设置成功')
            else:
                self.message_From('success')
        if name == 'default':
            if self.readOrNot is False:
                if self.chinese:
                    self.message_From('请先读取，否则无法获取舵机信息，从而进行设置！')
                else:
                    self.message_From('Read first！')
                return
            if self.chinese:
                if self.message_delect('此操作会对所有连接的舵机进行设置，请确保只连接了需要设置的舵机，是否继续？'):
                    return 
            else:
                if self.message_delect('This operation will set all the connected servos, please make sure whether the servo that needs to be set up is connected, whether to continue'):
                    return 
            self.readOrNot = False
            try:
                default_id = 1
                setServoID(254, default_id)
                time.sleep(0.01)
                data = getServoID()
                if data is not None:
                    if data[0] != default_id:
                        if self.chinese:
                            self.message_From('id设置失败！')
                        else:
                            self.message_From('Failed！')
                        return
                else:
                    if self.chinese:
                        self.message_From('id设置失败！')
                    else:
                        self.message_From('Failed！')
                    return
                setServoDeviation(default_id, 0)
                time.sleep(0.01)
                saveServoDeviation(default_id)
                time.sleep(0.01)
                data = getServoDeviation(default_id)
                if data is not None:
                    if data[0] != 0:
                        if self.chinese:
                            self.message_From('偏差设置失败！')
                        else:
                            self.message_From('Failed！')
                        return
                else:
                    if self.chinese:
                        self.message_From('偏差设置失败！')
                    else:
                        self.message_From('Failed！')
                    return
                setServoMaxTemp(default_id, 85)
                time.sleep(0.01)
                data = getServoTempLimit(default_id)
                if data is not None:
                    if data[0] != 85:
                        if self.chinese:
                            self.message_From('温度设置失败！')
                        else:
                            self.message_From('Failed！')
                        return
                else:
                    if self.chinese:
                        self.message_From('温度设置失败！')
                    else:
                        self.message_From('Failed！')
                    return
                setServoAngleLimit(1, 0, 1000)
                time.sleep(0.01)
                data = getServoAngleLimit(default_id)
                if data is not None:
                    if data != [0, 1000]:
                        if self.chinese:
                            self.message_From('角度范围设置失败！')
                        else:
                            self.message_From('Failed！')
                        return   
                else:
                    if self.chinese:
                        self.message_From('角度范围设置失败！')
                    else:
                        self.message_From('Failed！')
                    return
                setServoVinLimit(1, 4500, 14000)
                time.sleep(0.01)
                data = getServoVinLimit(default_id)
                if data is not None:
                    if data != [4500, 14000]:
                        if self.chinese:
                            self.message_From('电压范围设置失败！')
                        else:
                            self.message_From('Failed！')
                        return  
                else:
                    if self.chinese:
                        self.message_From('电压范围设置失败！')
                    else:
                        self.message_From('Failed！')
                    return

                setServoPulse(default_id, 500, 0)
            except:
                if self.chinese:
                    self.message_From('设置超时!')
                else:
                    self.message_From('Timeout!')
                return
            if self.chinese:
                self.message_From('设置成功')
            else:
                self.message_From('Success')
        if name == 'quit2':
            enable_reception(True)
            sys.exit()
        if name == 'resetPos':
            self.horizontalSlider_servoMove.setValue(500)
            setServoPulse(self.id, 500, 0)

    def button_clicked(self):
        button_object = self.sender()
        self.button_pressed = button_object
        if button_object in self.plus_object:
            self.plus_pressed = True
            self.start_time = time.time()
        elif button_object in self.reduce_object:
            self.reduce_pressed = True
            self.start_time = time.time()

    def button_released(self):
        button_object = self.sender()
        if time.time() - self.start_time <= 0.2:
            if self.plus_pressed:
                index = self.plus_object.index(self.button_pressed) + 1
                value = int(self.lineEdit_object[index - 1].text()) + 1
                self.slider_object[index - 1].setValue(value)
            elif self.reduce_pressed:
                index = self.reduce_object.index(self.button_pressed) + 1
                value = int(self.lineEdit_object[index - 1].text()) - 1
                self.slider_object[index - 1].setValue(value)    
        if button_object in self.plus_object:
            self.plus_pressed = False
        elif button_object in self.reduce_object:
            self.reduce_pressed = False

    def button_pressed_check(self):
        if self.plus_pressed:
            if time.time() - self.start_time > 0.2:
                index = self.plus_object.index(self.button_pressed) + 1
                value = int(self.lineEdit_object[index - 1].text()) + 1
                self.slider_object[index - 1].setValue(value)
        elif self.reduce_pressed:
            if time.time() - self.start_time > 0.2:
                index = self.reduce_object.index(self.button_pressed) + 1
                value = int(self.lineEdit_object[index - 1].text()) - 1
                self.slider_object[index - 1].setValue(value)           
        else:
            time.sleep(0.01)

if __name__ == "__main__":  
    app = QtWidgets.QApplication(sys.argv)
    myshow = MainWindow()
    myshow.show()
    sys.exit(app.exec_())
