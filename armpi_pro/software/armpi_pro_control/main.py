#!/usr/bin/env python3
# encoding: utf-8
import os
import re
import cv2
import sys
import copy
import math
import time
import sqlite3
import threading
import resource_rc
from socket import * 
from servo_controller import *
from ui import Ui_Form
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtSql import QSqlDatabase, QSqlQuery

class MainWindow(QtWidgets.QWidget, Ui_Form):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.set_window_position()
        
        self.servo_index = {'6':6, '5':5, '4':4, '3':3, '2':2, '1':1}
        self.horizontalSlider_index = {'1':1, '2':2, '3':3, '4':4, '5':5, '6':6} 
        self.path = os.path.split(os.path.realpath(__file__))[0]
        self.actdir = os.path.join(self.path, "ActionGroups")
        self.temp_path = os.path.join(self.actdir, 'temp.d6a')
        self.setWindowIcon(QIcon(os.path.join(self.path, 'resources/arm.png')))
        self.tabWidget.setCurrentIndex(0)  # 设置默认标签为第一个标签
        self.tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)  # 设置选中整行，若不设置默认选中单元格
        self.message = QMessageBox()
        self.joints = 6
        self.min_time = 20
        self.button_controlaction_clicked('reflash')
        ########################主界面###############################
        self.lineEdit_object = []    
        for i in range(self.joints):
            self.lineEdit_object.append(self.findChild(QLineEdit, "lineEdit_" + str(self.horizontalSlider_index[str(i + 1)]))) 
        self.validator = QIntValidator(0, 1000)
        for i in range(self.joints):
            self.lineEdit_object[i].setValidator(self.validator)

        self.horizontalSlider_pulse_object = []
        for i in range(self.joints):
            self.horizontalSlider_pulse_object.append(self.findChild(QSlider, "horizontalSlider_pulse_" + str(self.horizontalSlider_index[str(i + 1)])))
        # 滑竿同步对应文本框的数值,及滑竿控制相应舵机转动与valuechange函数绑定
        self.horizontalSlider_pulse_object[0].valueChanged.connect(lambda: self.valuechange1(0))
        self.horizontalSlider_pulse_object[1].valueChanged.connect(lambda: self.valuechange1(1))
        self.horizontalSlider_pulse_object[2].valueChanged.connect(lambda: self.valuechange1(2))
        self.horizontalSlider_pulse_object[3].valueChanged.connect(lambda: self.valuechange1(3))
        self.horizontalSlider_pulse_object[4].valueChanged.connect(lambda: self.valuechange1(4))
        self.horizontalSlider_pulse_object[5].valueChanged.connect(lambda: self.valuechange1(5))

        self.horizontalSlider_dev_object = []
        for i in range(self.joints):
            self.horizontalSlider_dev_object.append(self.findChild(QSlider, "horizontalSlider_dev_" + str(self.horizontalSlider_index[str(i + 1)]))) 
        self.horizontalSlider_dev_object[0].valueChanged.connect(lambda: self.valuechange2(0))
        self.horizontalSlider_dev_object[1].valueChanged.connect(lambda: self.valuechange2(1))
        self.horizontalSlider_dev_object[2].valueChanged.connect(lambda: self.valuechange2(2))
        self.horizontalSlider_dev_object[3].valueChanged.connect(lambda: self.valuechange2(3))
        self.horizontalSlider_dev_object[4].valueChanged.connect(lambda: self.valuechange2(4))
        self.horizontalSlider_dev_object[5].valueChanged.connect(lambda: self.valuechange2(5))
        
        self.label_object = []
        for i in range(self.joints):
            self.label_object.append(self.findChild(QLabel, "label_d" + str(self.horizontalSlider_index[str(i + 1)]))) 

        self.radioButton_zn.toggled.connect(lambda: self.language(self.radioButton_zn))
        self.radioButton_en.toggled.connect(lambda: self.language(self.radioButton_en))        
        self.chinese = True
        try:
            if os.environ['ASR_LANGUAGE'] == 'English':
                self.radioButton_en.setChecked(True)
                self.chinese = False
            else:
                self.radioButton_zn.setChecked(True)
        except:
            self.radioButton_zn.setChecked(True)
        
        # tableWidget点击获取定位的信号与icon_position函数（添加运行图标）绑定
        self.tableWidget.pressed.connect(self.icon_position)

        self.lineEdit_time.setValidator(QIntValidator(20, 30000))

        # 将编辑动作组的按钮点击时的信号与button_editaction_clicked函数绑定
        self.Button_ServoPowerDown.pressed.connect(lambda: self.button_editaction_clicked('servoPowerDown'))
        self.Button_AngularReadback.pressed.connect(lambda: self.button_editaction_clicked('angularReadback'))
        self.Button_AddAction.pressed.connect(lambda: self.button_editaction_clicked('addAction'))
        self.Button_DelectAction.pressed.connect(lambda: self.button_editaction_clicked('delectAction'))
        self.Button_DelectAllAction.pressed.connect(lambda: self.button_editaction_clicked('delectAllAction'))                                                 
        self.Button_UpdateAction.pressed.connect(lambda: self.button_editaction_clicked('updateAction'))
        self.Button_InsertAction.pressed.connect(lambda: self.button_editaction_clicked('insertAction'))
        self.Button_MoveUpAction.pressed.connect(lambda: self.button_editaction_clicked('moveUpAction'))
        self.Button_MoveDownAction.pressed.connect(lambda: self.button_editaction_clicked('moveDownAction'))        

        # 将运行及停止运行按钮点击的信号与button_runonline函数绑定
        self.Button_Run.clicked.connect(lambda: self.button_run('run'))

        self.Button_OpenActionGroup.pressed.connect(lambda: self.button_flie_operate('openActionGroup'))
        self.Button_SaveActionGroup.pressed.connect(lambda: self.button_flie_operate('saveActionGroup'))
        self.Button_ReadDeviation.pressed.connect(lambda: self.button_flie_operate('readDeviation'))
        self.Button_DownloadDeviation.pressed.connect(lambda: self.button_flie_operate('downloadDeviation'))
        self.Button_TandemActionGroup.pressed.connect(lambda: self.button_flie_operate('tandemActionGroup'))
        self.Button_ReSetServos.pressed.connect(lambda: self.button_re_clicked('reSetServos'))
        
        # 将控制动作的按钮点击的信号与action_control_clicked函数绑定
        self.Button_RunAction.pressed.connect(lambda: self.button_controlaction_clicked('runAction'))
        self.Button_StopAction.pressed.connect(lambda: self.button_controlaction_clicked('stopAction'))
        self.Button_Reflash.pressed.connect(lambda: self.button_controlaction_clicked('reflash'))
        self.Button_Quit.pressed.connect(lambda: self.button_controlaction_clicked('quit'))
        self.Button_run = QtWidgets.QToolButton()
        self.icon = QtGui.QIcon()
        self.icon.addPixmap(QtGui.QPixmap(os.path.join(self.path, "resources/index.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Button_run.setIcon(self.icon)

        for i in self.horizontalSlider_dev_object:
            i.setEnabled(False)
        
        self.save_temp = False
        self.devNew = [0, 0, 0, 0, 0, 0]
        self.dev_change = False 
        self.resetServos_ = False
        self.readDevOk = False
        self.totalTime = 0
        self.row = 0
        self.start_run = True
        self.use_time_list = []
        self.use_time_list_ = []
        self.loop = False
        self.running = False
        self.open_action = True
        self.editaction_clicked = False
        self.readOrNot = False
        enable_reception(False)

    def set_window_position(self):
        # 窗口居中
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def language(self, name):
        if self.radioButton_zn.isChecked() and name.text() == '中文':
            self.chinese = True
            self.Button_ServoPowerDown.setText("手掰编程")
            self.Button_AngularReadback.setText("角度回读")
            self.Button_AddAction.setText("添加动作")
            self.Button_DelectAction.setText("删除动作")
            self.Button_UpdateAction.setText("更新动作")
            self.Button_InsertAction.setText("插入动作")
            self.Button_MoveUpAction.setText("上移动作")
            self.Button_MoveDownAction.setText("下移动作")        
            self.Button_OpenActionGroup.setText("打开动作文件")
            self.Button_SaveActionGroup.setText("保存动作文件")
            self.Button_ReadDeviation.setText("读取偏差")
            self.Button_DownloadDeviation.setText("下载偏差")
            self.Button_TandemActionGroup.setText("串联动作文件")
            self.Button_ReSetServos.setText("舵机回中")
            self.Button_RunAction.setText("动作运行")
            self.Button_StopAction.setText("动作停止")
            self.Button_Run.setText("运行")
            self.checkBox.setText("循环")
            self.label_action.setText("动作组")
            self.label_time.setText("时间")
            self.label_time_2.setText("运行总时间")
            self.Button_Quit.setText("退出")
            self.Button_DelectAllAction.setText("删除全部")
            self.Button_Reflash.setText("刷新")
            self.label_open.setText("开")
            self.label_close.setText("合")
            self.label_up_2.setText("上")
            self.label_down_2.setText("下")
            self.label_up_3.setText("上")
            self.label_down_3.setText("下")
            self.label_up_1.setText("上")
            self.label_down_1.setText("下")
            self.label_left_1.setText("左")
            self.label_right_1.setText("右")
            self.label_left_2.setText("左")
            self.label_right_2.setText("右")
            item = self.tableWidget.horizontalHeaderItem(1)
            item.setText("编号")
            item = self.tableWidget.horizontalHeaderItem(2)
            item.setText("时间")
        elif self.radioButton_en.isChecked() and name.text() == 'English':
            self.chinese = False
            self.Button_ServoPowerDown.setText("Manual")
            self.Button_AngularReadback.setText("Read angle")
            self.Button_AddAction.setText("Add action")
            self.Button_DelectAction.setText("Delete action")
            self.Button_UpdateAction.setText("Update action")
            self.Button_InsertAction.setText("Insert action")
            self.Button_MoveUpAction.setText("Action upward")
            self.Button_MoveDownAction.setText("Action down")        
            self.Button_OpenActionGroup.setText("Open action file")
            self.Button_SaveActionGroup.setText("Save action file")
            self.Button_ReadDeviation.setText("Read deviation")
            self.Button_DownloadDeviation.setText("Download deviation")
            self.Button_TandemActionGroup.setText("Integrate file")
            self.Button_ReSetServos.setText("Reset servo")
            self.Button_RunAction.setText("Run action")
            self.Button_StopAction.setText("Stop")
            self.Button_Run.setText("Run")           
            self.checkBox.setText("Loop")
            self.label_action.setText("Action group")
            self.label_time.setText("Duration")
            self.label_time_2.setText("Total duration")  
            self.Button_Quit.setText("Quit")
            self.Button_DelectAllAction.setText("Delete all")
            self.Button_Reflash.setText("Reflash")
            self.label_open.setText("Open")
            self.label_close.setText("Close")
            self.label_up_2.setText("Up")
            self.label_down_2.setText("Down")
            self.label_up_3.setText("Up")
            self.label_down_3.setText("Down")
            self.label_up_1.setText("Up")
            self.label_down_1.setText("Down")
            self.label_left_1.setText("Left")
            self.label_right_1.setText("Right")
            self.label_left_2.setText("Left")
            self.label_right_2.setText("Right")
            item = self.tableWidget.horizontalHeaderItem(1)
            item.setText("Index")
            item = self.tableWidget.horizontalHeaderItem(2)
            item.setText("Time")

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
            self.camera_ui = True
            self.camera_ui_break = True
            QWidget.closeEvent(self, e)
        else:
            e.ignore()
    
    def keyPressEvent(self, event):
        if event.key() == 16777220 or event.key() == 16777221:
            self.resetServos_ = True
            for i in range(self.joints):
                pulse = int(self.lineEdit_object[i].text())
                self.horizontalSlider_pulse_object[i].setValue(pulse)
                setServoPulse(self.servo_index[str(i + 1)], pulse, 500)
            self.resetServos_ = False
    
    def tabchange(self):
        if self.tabWidget.currentIndex() == 1:
            if self.chinese:
                self.message_From('使用此面板时，请确保只连接了一个舵机，否则会引起冲突！')
            else:
                self.message_From('Before debugging servo,make sure that the servo controller is connected with ONE servo.Otherwise it may cause a conflict!')
        
    
    # 滑竿同步对应文本框的数值,及滑竿控制相应舵机转动
    def valuechange1(self, name):
        if not self.resetServos_:
            servo_pulse = self.horizontalSlider_pulse_object[name].value()
            setServoPulse(self.servo_index[str(name + 1)], servo_pulse, self.min_time)
            self.lineEdit_object[name].setText(str(servo_pulse))    

    def valuechange2(self, name):
        if self.readDevOk:
            self.devNew[0] = self.horizontalSlider_dev_object[name].value()
            setServoDeviation(self.servo_index[str(name + 1)], self.devNew[0])
            self.label_object[name].setText(str(self.devNew[0]))
            if self.devNew[0] < 0:
                self.devNew[0] = 0xff + self.devNew[0] + 1 
        else:
            self.message_From('请先读取偏差!')
                     
    # 复位按钮点击事件
    def button_re_clicked(self, name):
        self.resetServos_ = True
        if name == 'reSetServos':
            for i in range(self.joints):
                self.horizontalSlider_pulse_object[i].setValue(500)
                setServoPulse(self.servo_index[str(i + 1)], 500, 2000)
                self.lineEdit_object[i].setText('500')
            self.resetServos_ = False

    # 选项卡选择标签状态，获取对应舵机数值
    def tabindex(self, index):       
        return  [str(self.horizontalSlider_pulse_object[i].value()) for i in range(self.joints)]
    
    def getIndexData(self, index):
        return [str(self.tableWidget.item(index, j).text()) for j in range(2, self.tableWidget.columnCount())]
    
    # 往tableWidget表格添加一行数据的函数
    def add_line(self, data):
        self.tableWidget.setItem(data[0], 1, QtWidgets.QTableWidgetItem(str(data[0] + 1)))       
        for i in range(2, len(data) + 1):          
            self.tableWidget.setItem(data[0], i, QtWidgets.QTableWidgetItem(data[i - 1]))

    # 在定位行添加运行图标按钮
    def icon_position(self):
        if self.tableWidget.currentColumn() == 0:
            if not self.running:
                self.Button_run = QtWidgets.QToolButton()
                self.Button_run.setIcon(self.icon) 
                item = self.tableWidget.currentRow()
                for i in range(self.tableWidget.rowCount()):
                    if i != item:
                        self.tableWidget.removeCellWidget(i, 0)
                if self.open_action:
                    self.open_action = False
                else:
                    if self.editaction_clicked:
                        self.editaction_clicked = False
                        self.tableWidget.setCellWidget(item, 0, self.Button_run)
                        self.Button_run.clicked.connect(self.icon_position)
                    else:
                        self.tableWidget.setCellWidget(item, 0, self.Button_run)
                        self.Button_run.clicked.connect(self.icon_position)
                        self.action_one()

    def action_one(self):
        self.resetServos_ = True
        item = self.tableWidget.currentRow()
        try:
            timer = int(self.tableWidget.item(self.tableWidget.currentRow(), 2).text())
            for ID in range(1, self.joints + 1):
                pulse = int(self.tableWidget.item(item, ID + 2).text())
                self.horizontalSlider_pulse_object[ID - 1].setValue(pulse)
                self.lineEdit_object[ID - 1].setText(str(pulse))
                setServoPulse(ID, pulse, timer) 
        except BaseException as e:
            print(e)
            if self.chinese:
                self.message_From('运行出错')
            else:
                self.message_From('Running error')
        self.resetServos_ = False

    # 编辑动作组按钮点击事件
    def button_editaction_clicked(self, name):
        self.editaction_clicked = True
        list_data = self.tabindex(self.tableWidget.currentIndex())
        RowCont = self.tableWidget.rowCount()
        item = self.tableWidget.currentRow()
        if name == 'servoPowerDown':
            for servo_id in range(1, self.joints + 1):
                unloadServo(servo_id)
            if self.chinese:
                self.message_From('掉电成功')
            else:
                self.message_From('success')
        if name == 'angularReadback':
            self.tableWidget.insertRow(RowCont)    # 增加一行
            self.tableWidget.selectRow(RowCont)    # 定位最后一行为选中行
            use_time = int(self.lineEdit_time.text())
            data = [RowCont, str(use_time)]
            for i in range(1, self.joints + 1):
                pulse = getServoPulse(self.servo_index[str(i)])
                if pulse is None:
                    return
                else:
                    data.append(str(pulse))                                       
            if use_time < 20:
                if self.chinese:
                    self.message_From('运行时间必须大于20ms')
                else:
                    self.message_From('Run time must be greater than 20ms')
                return        
            self.add_line(data)
            self.totalTime += use_time
            self.label_TotalTime.setText(str((self.totalTime)/1000.0))            
        if name == 'addAction':    # 添加动作
            use_time = int(self.lineEdit_time.text())
            data = [RowCont, str(use_time)]
            if use_time < 20:
                if self.chinese:
                    self.message_From('运行时间必须大于20')
                else:
                    self.message_From('Run time must greater than 20')
                return
            self.tableWidget.insertRow(RowCont)    # 增加一行
            self.tableWidget.selectRow(RowCont)    # 定位最后一行为选中行
            data.extend(list_data)
            self.add_line(data)
            self.totalTime += int(self.lineEdit_time.text())
            self.label_TotalTime.setText(str((self.totalTime)/1000.0))
        if name == 'delectAction':    # 删除动作
            if RowCont != 0:
                self.totalTime -= int(self.tableWidget.item(item, 2).text())
                self.tableWidget.removeRow(item)  # 删除选定行                
                self.label_TotalTime.setText(str((self.totalTime)/1000.0))
        if name == 'delectAllAction':
            result = self.message_delect('此操作会删除列表中的所有动作，是否继续？')
            if result == 0:                              
                for i in range(RowCont):
                    self.tableWidget.removeRow(0)
                self.totalTime = 0
                self.label_TotalTime.setText(str(self.totalTime))
            else:
                pass          
        if name == 'updateAction':    # 更新动作
            use_time = int(self.lineEdit_time.text())
            data = [item, str(use_time)]            
            if use_time < 20:
                if self.chinese:
                    self.message_From('运行时间必须大于20')
                else:
                    self.message_From('Run time must greater than 20')
                return

            data.extend(list_data)
            self.add_line(data)
            self.totalTime = 0
            for i in range(RowCont):
                self.totalTime += int(self.tableWidget.item(i,2).text())
            self.label_TotalTime.setText(str((self.totalTime)/1000.0))            
        if name == 'insertAction':    # 插入动作
            if item == -1:
                return
            use_time = int(self.lineEdit_time.text())
            data = [item, str(use_time)]            
            if use_time < 20:
                if self.chinese:
                    self.message_From('运行时间必须大于20')
                else:
                    self.message_From('Run time must greater than 20')
                return

            self.tableWidget.insertRow(item)       # 插入一行
            self.tableWidget.selectRow(item)
            data.extend(list_data)
            self.add_line(data)
            self.totalTime += int(self.lineEdit_time.text())
            self.label_TotalTime.setText(str((self.totalTime)/1000.0))
        if name == 'moveUpAction':
            data_new = [item - 1]
            data = [item]
            if item == 0 or item == -1:
                return
            current_data = self.getIndexData(item)
            uplist_data = self.getIndexData(item - 1)
            data_new.extend(current_data)
            data.extend(uplist_data)
            self.add_line(data_new)           
            self.add_line(data)
            self.tableWidget.selectRow(item - 1) 
        if name == 'moveDownAction':
            data_new = [item + 1]
            data = [item]   
            if item == RowCont - 1:
                return
            current_data = self.getIndexData(item)
            downlist_data = self.getIndexData(item + 1)           
            data_new.extend(current_data)
            data.extend(downlist_data)
            self.add_line(data_new)           
            self.add_line(data) 
            self.tableWidget.selectRow(item + 1)
                             
        for i in range(self.tableWidget.rowCount()):    #刷新编号值
            self.tableWidget.item(i , 2).setFlags(self.tableWidget.item(i , 2).flags() & ~Qt.ItemIsEditable)
            self.tableWidget.setItem(i,1,QtWidgets.QTableWidgetItem(str(i + 1)))
        self.icon_position()

    # 在线运行按钮点击事件
    def button_run(self, name):
        if self.tableWidget.rowCount() == 0:
            if self.chinese:
                self.message_from('请先添加动作!')
            else:
                self.message_from('Please add action first!')
        else:
            if name == 'run':
                try:
                    if not self.running:
                        self.save_temp = True
                        self.button_flie_operate('saveActionGroup')
                        self.save_temp = False
                        self.running = True
                        if self.Button_Run.text() == 'Run':
                            self.Button_Run.setText('Stop')
                        else:
                            self.Button_Run.setText('停止')                        
                        
                        self.row = self.tableWidget.currentRow()
                        if self.checkBox.isChecked():
                            self.loop = True
                        else:
                            self.loop = False

                        self.start_run = True
                        self.timer = QTimer()      
                        self.timer.setTimerType(Qt.PreciseTimer)
                        self.timer.timeout.connect(self.runOline)
                        self.use_time_list = []
                        for i in range(self.tableWidget.rowCount() - self.row):
                            use_time = int(self.tableWidget.item(i, 2).text())
                            self.use_time_list.append(use_time)
                        self.use_time_list_ = copy.deepcopy(self.use_time_list)
                        self.timer.start(100)
                    else:
                        for i in range(self.tableWidget.currentRow()):
                            if i != self.tableWidget.currentRow():
                                self.tableWidget.removeCellWidget(i, 0)
                        self.timer.stop()
                        self.running = False
                        if self.Button_Run.text() == 'Stop':
                            self.Button_Run.setText('Run')
                        else:
                            self.Button_Run.setText('运行')
                        if self.chinese:
                            self.message_from('运行结束!')
                        else:
                            self.message_from('Run over!')
                        self.Button_run = QtWidgets.QToolButton()
                        self.Button_run.setIcon(self.icon)
                        self.tableWidget.setCellWidget(self.tableWidget.currentRow(), 0, self.Button_run)
                except BaseException as e:
                    print(e)   

    def runOline(self):
        if self.use_time_list_ == []:
            if self.loop:
                self.use_time_list_ = copy.deepcopy(self.use_time_list)

                self.tableWidget.removeCellWidget(self.tableWidget.currentRow(), 0)
                self.Button_run = QtWidgets.QToolButton()
                self.Button_run.setIcon(self.icon)
                self.tableWidget.selectRow(self.row)
                self.tableWidget.setCellWidget(self.row, 0, self.Button_run)
                
                item = self.tableWidget.currentRow()
                timer = int(self.tableWidget.item(self.tableWidget.currentRow(), 2).text())
                data = [timer]
                for ID in range(1, self.joints + 1):
                    pulse = int(self.tableWidget.item(item, ID + 2).text())
                    setServoPulse(self.servo_index[str(ID)], pulse, timer)
                self.timer.start(self.use_time_list_[0])
                self.start_run = False
                self.use_time_list_.remove(self.use_time_list_[0])
            else:
                self.tableWidget.removeCellWidget(self.tableWidget.currentRow(), 0)
                self.timer.stop()
                self.running = False
                if self.Button_Run.text() == 'Stop':
                    self.Button_Run.setText('Run')
                else:
                    self.Button_Run.setText('运行')
                if self.chinese:
                    self.message_from('运行结束!')
                else:
                    self.message_from('Run over!')
                self.Button_run = QtWidgets.QToolButton()
                self.Button_run.setIcon(self.icon)
                self.tableWidget.selectRow(self.row)
                 
                self.tableWidget.setCellWidget(self.row, 0, self.Button_run)
        else:
            if not self.start_run:
                self.tableWidget.selectRow(self.tableWidget.currentRow() + 1)
            self.tableWidget.setCellWidget(self.tableWidget.currentRow(), 0, self.Button_run)       
            item = self.tableWidget.currentRow()
            timer = int(self.tableWidget.item(self.tableWidget.currentRow(), 2).text())
            data = [timer]
            for ID in range(1, self.joints + 1):
                pulse = int(self.tableWidget.item(item, ID + 2).text())
                setServoPulse(self.servo_index[str(ID)], pulse, timer)
            self.timer.start(self.use_time_list_[0])
            self.start_run = False
            self.use_time_list_.remove(self.use_time_list_[0])

    def action_online(self, item):
        try:
            item = self.tableWidget.currentRow()
            timer = int(self.tableWidget.item(self.tableWidget.currentRow(), 2).text())
            for servo_id in range(1, self.joints + 1):
                pulse = int(self.tableWidget.item(item, servo_id + 2).text())
                setServoPulse(self.servo_index[str(ID)], pulse, timer)
        except BaseException as e:
            print(e)
            self.timer.stop()
            if self.chinese:
                self.Button_Run.setText('运行')
                self.message_From('运行出错!')
            else:
                self.Button_Run.setText('Run')
                self.message_From('Run error!')              

    # 文件打开及保存按钮点击事件
    def button_flie_operate(self, name):
        self.open_action = True
        try:            
            if name == 'openActionGroup':
                dig_o = QFileDialog()
                dig_o.setFileMode(QFileDialog.ExistingFile)
                dig_o.setNameFilter('d6a Flies(*.d6a)')
                openfile = dig_o.getOpenFileName(self, 'OpenFile', self.actdir, 'd6a Flies(*.d6a)')
                # 打开单个文件
                # 参数一：设置父组件；参数二：QFileDialog的标题
                # 参数三：默认打开的目录，“.”点表示程序运行目录，/表示当前盘符根目录
                # 参数四：对话框的文件扩展名过滤器Filter，比如使用 Image files(*.jpg *.gif) 表示只能显示扩展名为.jpg或者.gif文件
                # 设置多个文件扩展名过滤，使用双引号隔开；“All Files(*);;PDF Files(*.pdf);;Text Files(*.txt)”
                path = openfile[0]
                try:
                    if path != '':
                        rbt = QSqlDatabase.addDatabase("QSQLITE")
                        rbt.setDatabaseName(path)
                        if rbt.open():
                            actgrp = QSqlQuery()
                            if (actgrp.exec("select * from ActionGroup ")):
                                self.tableWidget.setRowCount(0)
                                self.tableWidget.clearContents()
                                self.totalTime = 0
                                while (actgrp.next()):
                                    count = self.tableWidget.rowCount()
                                    self.tableWidget.setRowCount(count + 1)
                                    for i in range(self.joints + 2):
                                        self.tableWidget.setItem(count, i + 1, QtWidgets.QTableWidgetItem(str(actgrp.value(i))))
                                        if i == 1:
                                            self.totalTime += actgrp.value(i)
                                        self.tableWidget.update()
                                        self.tableWidget.selectRow(count)
                                    self.tableWidget.item(count , 2).setFlags(self.tableWidget.item(count , 2).flags() & ~Qt.ItemIsEditable)                                        
                        self.icon_position()
                        rbt.close()
                        self.label_TotalTime.setText(str(self.totalTime/1000.0))
                except:
                    if self.chinese:
                        self.message_From('动作组错误')
                    else:
                        self.message_From('Wrong action format')
                self.action_group_name.setText(str(path))  
            if name == 'saveActionGroup':
                if not self.save_temp:
                    dig_s = QFileDialog()
                    if self.tableWidget.rowCount() == 0:
                        if self.chinese:
                            self.message_from('动作列表是空的哦，没啥要保存的')
                        else:
                            self.message_from('The action list is empty, nothing to save')                      
                        return
                    action_group_name = self.action_group_name.text()
                    if action_group_name == '':
                        action_group_name = self.actdir
                    savefile = dig_s.getSaveFileName(self, 'Savefile', action_group_name, 'd6a Flies(*.d6a)')
                    path = savefile[0]
                else:
                    path = self.temp_path
                if os.path.isfile(path):
                    os.system('sudo rm ' + path)
                if path != '':                    
                    if path[-4:] == '.d6a':
                        conn = sqlite3.connect(path)
                    else:
                        conn = sqlite3.connect(path + '.d6a')
                    
                    c = conn.cursor()                    
                    c.execute('''CREATE TABLE ActionGroup([Index] INTEGER PRIMARY KEY AUTOINCREMENT
                    NOT NULL ON CONFLICT FAIL
                    UNIQUE ON CONFLICT ABORT,
                    Time INT,
                    Servo1 INT,
                    Servo2 INT,
                    Servo3 INT,
                    Servo4 INT,
                    Servo5 INT,
                    Servo6 INT);''')                      
                    for i in range(self.tableWidget.rowCount()):
                        insert_sql = "INSERT INTO ActionGroup(Time, Servo1, Servo2, Servo3, Servo4, Servo5, Servo6) VALUES("
                        for j in range(2, self.tableWidget.columnCount()):
                            if j == self.tableWidget.columnCount() - 1:
                                insert_sql += str(self.tableWidget.item(i, j).text())
                            else:
                                insert_sql += str(self.tableWidget.item(i, j).text()) + ','
                        
                        insert_sql += ");"
                        c.execute(insert_sql)
                    
                    conn.commit()
                    conn.close()
                    self.button_controlaction_clicked('reflash')
            if name == 'readDeviation':
                for i in self.horizontalSlider_dev_object:
                    i.setEnabled(True)
                servo_id = ''
                self.readDevOk = True
                dev_data = []
                for i in range(1, self.joints + 1):
                    dev = getServoDeviation(self.servo_index[str(i)])
                    if dev is None:
                        dev_data.append(0)
                        servo_id += (' id' + str(i))
                    elif dev > 125:  # 负数
                        dev_data.append(-(0xff - (dev - 1)))                        
                    else:
                        dev_data.append(dev)
                for i in range(self.joints):
                    self.horizontalSlider_dev_object[i].setValue(dev_data[i])
                    self.label_object[i].setText(str(dev_data[i]))
                    
                if servo_id == '':
                    if self.chinese:
                        self.message_From('读取偏差成功!')
                    else:
                        self.message_From('success!')
                else:
                    if self.chinese:
                        self.message_From(servo_id + '号舵机偏差读取失败!')
                    else:
                        self.message_From('Failed to read the deviation of' + servo_id)
            if name == 'downloadDeviation':
                if self.readDevOk:                    
                    for servo_id in range(1, self.joints + 1):
                        saveServoDeviation(self.servo_index[str(servo_id)])
                    if self.chinese:
                        self.message_From('下载偏差成功!')
                    else:
                        self.message_From('success!')
                else:
                    if self.chinese:
                        self.message_From('请先读取偏差！')
                    else:
                        self.message_From('Please read the deviation first！')
                self.readDevOK = False
                for i in self.horizontalSlider_dev_object:
                    i.setEnabled(False)
            if name == 'tandemActionGroup':
                dig_t = QFileDialog()
                dig_t.setFileMode(QFileDialog.ExistingFile)
                dig_t.setNameFilter('d6a Flies(*.d6a)')
                openfile = dig_t.getOpenFileName(self, 'OpenFile', self.actdir, 'd6a Flies(*.d6a)')
                # 打开单个文件
                # 参数一：设置父组件；参数二：QFileDialog的标题
                # 参数三：默认打开的目录，“.”点表示程序运行目录，/表示当前盘符根目录
                # 参数四：对话框的文件扩展名过滤器Filter，比如使用 Image files(*.jpg *.gif) 表示只能显示扩展名为.jpg或者.gif文件
                # 设置多个文件扩展名过滤，使用双引号隔开；“All Files(*);;PDF Files(*.pdf);;Text Files(*.txt)”
                path = openfile[0]
                try:
                    if path != '':
                        tbt = QSqlDatabase.addDatabase("QSQLITE")
                        tbt.setDatabaseName(path)
                        if tbt.open():
                            actgrp = QSqlQuery()
                            if (actgrp.exec("select * from ActionGroup ")):
                                while (actgrp.next()):
                                    count = self.tableWidget.rowCount()
                                    self.tableWidget.setRowCount(count + 1)
                                    for i in range(self.joints + 2):
                                        if i == 0:
                                            self.tableWidget.setItem(count, i + 1, QtWidgets.QTableWidgetItem(str(count + 1)))
                                        else:                      
                                            self.tableWidget.setItem(count, i + 1, QtWidgets.QTableWidgetItem(str(actgrp.value(i))))
                                        if i == 1:
                                            self.totalTime += actgrp.value(i)
                                        self.tableWidget.update()
                                        self.tableWidget.selectRow(count)
                                    self.tableWidget.item(count , 2).setFlags(self.tableWidget.item(count , 2).flags() & ~Qt.ItemIsEditable)
                        self.icon_position()
                        tbt.close()
                        self.label_TotalTime.setText(str(self.totalTime/1000.0))
                except:
                    if self.chinese:
                        self.message_From('动作组错误')
                    else:
                        self.message_From('Wrong action format')
        except BaseException as e:
            print(e)

    def listActions(self, path):
        if not os.path.exists(path):
            os.mkdir(path)
        pathlist = os.listdir(path)
        actList = []
        
        for f in pathlist:
            if f[0] == '.':
                pass
            else:
                if f[-4:] == '.d6a':
                    f.replace('-', '')
                    if f:
                        actList.append(f[0:-4])
                else:
                    pass
        return actList
    
    def reflash_action(self):
        actList = self.listActions(self.actdir)
        actList.sort()
        
        if len(actList) != 0:        
            self.comboBox_action.clear()
            for i in range(0, len(actList)):
                self.comboBox_action.addItem(actList[i])
        else:
            self.comboBox_action.clear()
    
    # 控制动作组按钮点击事件
    def button_controlaction_clicked(self, name):
        if name == 'runAction':   # 动作组运行
            runActionGroup(self.comboBox_action.currentText())            
        if name == 'stopAction':   # 停止运行
            stopActionGroup()
        if name == 'reflash':
            self.reflash_action()
        if name == 'quit':
            enable_reception(True)
            sys.exit()

if __name__ == "__main__":  
    app = QtWidgets.QApplication(sys.argv)
    myshow = MainWindow()
    myshow.show()
    sys.exit(app.exec_())
