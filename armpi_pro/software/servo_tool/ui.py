# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.setWindowModality(QtCore.Qt.NonModal)
        Form.setEnabled(True)
        Form.resize(1200, 620)
        Form.setMinimumSize(QtCore.QSize(0, 0))
        Form.setMaximumSize(QtCore.QSize(1200, 620))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(11)
        Form.setFont(font)
        Form.setFocusPolicy(QtCore.Qt.NoFocus)
        Form.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        Form.setAcceptDrops(False)
        Form.setAutoFillBackground(False)
        Form.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.tabWidget = QtWidgets.QTabWidget(Form)
        self.tabWidget.setGeometry(QtCore.QRect(0, 10, 1200, 600))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.tabWidget.setFont(font)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.pushButton_read = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_read.setGeometry(QtCore.QRect(230, 490, 100, 50))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(16)
        self.pushButton_read.setFont(font)
        self.pushButton_read.setStyleSheet("QPushButton{background-color: rgb(255, 165, 0);}\n"
"QPushButton:hover{background-color:  rgb(255, 210, 0);}\n"
"QPushButton{border-radius:5px;}")
        self.pushButton_read.setObjectName("pushButton_read")
        self.pushButton_set = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_set.setGeometry(QtCore.QRect(430, 490, 100, 50))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(16)
        self.pushButton_set.setFont(font)
        self.pushButton_set.setStyleSheet("QPushButton{background-color: rgb(255, 165, 0);}\n"
"QPushButton:hover{background-color:  rgb(255, 210, 0);}\n"
"QPushButton{border-radius:5px;}\n"
"QPushButton{color: rgb(0, 0, 0);}")
        self.pushButton_set.setObjectName("pushButton_set")
        self.pushButton_default = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_default.setGeometry(QtCore.QRect(630, 490, 100, 50))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(16)
        self.pushButton_default.setFont(font)
        self.pushButton_default.setStyleSheet("QPushButton{background-color: rgb(255, 165, 0);}\n"
"QPushButton:hover{background-color:  rgb(255, 210, 0);}\n"
"QPushButton{border-radius:5px;}\n"
"QPushButton{color: rgb(0, 0, 0);}")
        self.pushButton_default.setObjectName("pushButton_default")
        self.widget = QtWidgets.QWidget(self.tab_2)
        self.widget.setGeometry(QtCore.QRect(460, 90, 220, 160))
        self.widget.setStyleSheet("#widget {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget.setObjectName("widget")
        self.label_angle = QtWidgets.QLabel(self.widget)
        self.label_angle.setGeometry(QtCore.QRect(63, 10, 101, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_angle.setFont(font)
        self.label_angle.setAlignment(QtCore.Qt.AlignCenter)
        self.label_angle.setObjectName("label_angle")
        self.horizontalSlider_servoMin = QtWidgets.QSlider(self.widget)
        self.horizontalSlider_servoMin.setGeometry(QtCore.QRect(30, 60, 160, 35))
        self.horizontalSlider_servoMin.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider_servoMin.setMaximum(1000)
        self.horizontalSlider_servoMin.setPageStep(1)
        self.horizontalSlider_servoMin.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servoMin.setObjectName("horizontalSlider_servoMin")
        self.horizontalSlider_servoMax = QtWidgets.QSlider(self.widget)
        self.horizontalSlider_servoMax.setGeometry(QtCore.QRect(30, 120, 160, 35))
        self.horizontalSlider_servoMax.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider_servoMax.setMinimum(0)
        self.horizontalSlider_servoMax.setMaximum(1000)
        self.horizontalSlider_servoMax.setPageStep(1)
        self.horizontalSlider_servoMax.setProperty("value", 1000)
        self.horizontalSlider_servoMax.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servoMax.setObjectName("horizontalSlider_servoMax")
        self.label_servoMin = QtWidgets.QLabel(self.widget)
        self.label_servoMin.setGeometry(QtCore.QRect(15, 30, 41, 31))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoMin.setFont(font)
        self.label_servoMin.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servoMin.setObjectName("label_servoMin")
        self.label_servoMax = QtWidgets.QLabel(self.widget)
        self.label_servoMax.setGeometry(QtCore.QRect(160, 90, 41, 31))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoMax.setFont(font)
        self.label_servoMax.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servoMax.setObjectName("label_servoMax")
        self.widget_6 = QtWidgets.QWidget(self.tab_2)
        self.widget_6.setGeometry(QtCore.QRect(460, 280, 220, 160))
        self.widget_6.setStyleSheet("#widget_6 {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget_6.setObjectName("widget_6")
        self.label_vin = QtWidgets.QLabel(self.widget_6)
        self.label_vin.setGeometry(QtCore.QRect(63, 10, 111, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_vin.setFont(font)
        self.label_vin.setAlignment(QtCore.Qt.AlignCenter)
        self.label_vin.setObjectName("label_vin")
        self.horizontalSlider_servoMinV = QtWidgets.QSlider(self.widget_6)
        self.horizontalSlider_servoMinV.setGeometry(QtCore.QRect(30, 60, 160, 35))
        self.horizontalSlider_servoMinV.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider_servoMinV.setMinimum(45)
        self.horizontalSlider_servoMinV.setMaximum(140)
        self.horizontalSlider_servoMinV.setPageStep(1)
        self.horizontalSlider_servoMinV.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servoMinV.setObjectName("horizontalSlider_servoMinV")
        self.horizontalSlider_servoMaxV = QtWidgets.QSlider(self.widget_6)
        self.horizontalSlider_servoMaxV.setGeometry(QtCore.QRect(30, 120, 160, 35))
        self.horizontalSlider_servoMaxV.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider_servoMaxV.setMinimum(45)
        self.horizontalSlider_servoMaxV.setMaximum(140)
        self.horizontalSlider_servoMaxV.setPageStep(1)
        self.horizontalSlider_servoMaxV.setProperty("value", 140)
        self.horizontalSlider_servoMaxV.setSliderPosition(140)
        self.horizontalSlider_servoMaxV.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servoMaxV.setObjectName("horizontalSlider_servoMaxV")
        self.label_servoMinV = QtWidgets.QLabel(self.widget_6)
        self.label_servoMinV.setGeometry(QtCore.QRect(20, 30, 50, 31))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoMinV.setFont(font)
        self.label_servoMinV.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servoMinV.setObjectName("label_servoMinV")
        self.label_servoMaxV = QtWidgets.QLabel(self.widget_6)
        self.label_servoMaxV.setGeometry(QtCore.QRect(150, 90, 50, 31))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoMaxV.setFont(font)
        self.label_servoMaxV.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servoMaxV.setObjectName("label_servoMaxV")
        self.widget_7 = QtWidgets.QWidget(self.tab_2)
        self.widget_7.setGeometry(QtCore.QRect(100, 350, 220, 90))
        self.widget_7.setStyleSheet("#widget_7 {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget_7.setObjectName("widget_7")
        self.horizontalSlider_servoTemp = QtWidgets.QSlider(self.widget_7)
        self.horizontalSlider_servoTemp.setGeometry(QtCore.QRect(50, 43, 160, 35))
        self.horizontalSlider_servoTemp.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider_servoTemp.setMinimum(50)
        self.horizontalSlider_servoTemp.setMaximum(100)
        self.horizontalSlider_servoTemp.setPageStep(1)
        self.horizontalSlider_servoTemp.setProperty("value", 85)
        self.horizontalSlider_servoTemp.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servoTemp.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_servoTemp.setObjectName("horizontalSlider_servoTemp")
        self.label_temp = QtWidgets.QLabel(self.widget_7)
        self.label_temp.setGeometry(QtCore.QRect(58, 10, 111, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_temp.setFont(font)
        self.label_temp.setAlignment(QtCore.Qt.AlignCenter)
        self.label_temp.setObjectName("label_temp")
        self.label_servoTemp = QtWidgets.QLabel(self.widget_7)
        self.label_servoTemp.setGeometry(QtCore.QRect(4, 45, 45, 30))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoTemp.setFont(font)
        self.label_servoTemp.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servoTemp.setObjectName("label_servoTemp")
        self.widget_8 = QtWidgets.QWidget(self.tab_2)
        self.widget_8.setGeometry(QtCore.QRect(810, 90, 291, 180))
        self.widget_8.setStyleSheet("#widget_8 {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget_8.setObjectName("widget_8")
        self.label_current_angle = QtWidgets.QLabel(self.widget_8)
        self.label_current_angle.setGeometry(QtCore.QRect(15, 60, 171, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_current_angle.setFont(font)
        self.label_current_angle.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_current_angle.setObjectName("label_current_angle")
        self.label_current_temp = QtWidgets.QLabel(self.widget_8)
        self.label_current_temp.setGeometry(QtCore.QRect(15, 140, 171, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_current_temp.setFont(font)
        self.label_current_temp.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_current_temp.setObjectName("label_current_temp")
        self.label_current_vin = QtWidgets.QLabel(self.widget_8)
        self.label_current_vin.setGeometry(QtCore.QRect(15, 100, 171, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_current_vin.setFont(font)
        self.label_current_vin.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_current_vin.setObjectName("label_current_vin")
        self.label_servoCurrentP = QtWidgets.QLabel(self.widget_8)
        self.label_servoCurrentP.setGeometry(QtCore.QRect(210, 60, 60, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoCurrentP.setFont(font)
        self.label_servoCurrentP.setText("")
        self.label_servoCurrentP.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_servoCurrentP.setObjectName("label_servoCurrentP")
        self.label_servoCurrentV = QtWidgets.QLabel(self.widget_8)
        self.label_servoCurrentV.setGeometry(QtCore.QRect(210, 100, 60, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoCurrentV.setFont(font)
        self.label_servoCurrentV.setText("")
        self.label_servoCurrentV.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_servoCurrentV.setObjectName("label_servoCurrentV")
        self.label_servoCurrentTemp = QtWidgets.QLabel(self.widget_8)
        self.label_servoCurrentTemp.setGeometry(QtCore.QRect(210, 140, 60, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoCurrentTemp.setFont(font)
        self.label_servoCurrentTemp.setText("")
        self.label_servoCurrentTemp.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_servoCurrentTemp.setObjectName("label_servoCurrentTemp")
        self.label_current_id = QtWidgets.QLabel(self.widget_8)
        self.label_current_id.setGeometry(QtCore.QRect(15, 20, 171, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_current_id.setFont(font)
        self.label_current_id.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_current_id.setObjectName("label_current_id")
        self.label_servoCurrentID = QtWidgets.QLabel(self.widget_8)
        self.label_servoCurrentID.setGeometry(QtCore.QRect(210, 20, 60, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoCurrentID.setFont(font)
        self.label_servoCurrentID.setText("")
        self.label_servoCurrentID.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_servoCurrentID.setObjectName("label_servoCurrentID")
        self.widget_9 = QtWidgets.QWidget(self.tab_2)
        self.widget_9.setGeometry(QtCore.QRect(100, 90, 220, 101))
        self.widget_9.setStyleSheet("#widget_9 {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget_9.setObjectName("widget_9")
        self.label_id = QtWidgets.QLabel(self.widget_9)
        self.label_id.setGeometry(QtCore.QRect(60, 5, 101, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_id.setFont(font)
        self.label_id.setAlignment(QtCore.Qt.AlignCenter)
        self.label_id.setObjectName("label_id")
        self.lineEdit1 = QtWidgets.QLineEdit(self.widget_9)
        self.lineEdit1.setGeometry(QtCore.QRect(50, 40, 130, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.lineEdit1.setFont(font)
        self.lineEdit1.setMaxLength(255)
        self.lineEdit1.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit1.setObjectName("lineEdit1")
        self.horizontalSlider1 = QtWidgets.QSlider(self.widget_9)
        self.horizontalSlider1.setGeometry(QtCore.QRect(45, 65, 135, 35))
        self.horizontalSlider1.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider1.setMinimum(-1)
        self.horizontalSlider1.setMaximum(255)
        self.horizontalSlider1.setPageStep(1)
        self.horizontalSlider1.setProperty("value", 1)
        self.horizontalSlider1.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider1.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider1.setObjectName("horizontalSlider1")
        self.pushButton_reduce1 = QtWidgets.QPushButton(self.widget_9)
        self.pushButton_reduce1.setGeometry(QtCore.QRect(10, 65, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(30)
        self.pushButton_reduce1.setFont(font)
        self.pushButton_reduce1.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_reduce1.setObjectName("pushButton_reduce1")
        self.pushButton_plus1 = QtWidgets.QPushButton(self.widget_9)
        self.pushButton_plus1.setGeometry(QtCore.QRect(185, 65, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(25)
        self.pushButton_plus1.setFont(font)
        self.pushButton_plus1.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_plus1.setObjectName("pushButton_plus1")
        self.widget_10 = QtWidgets.QWidget(self.tab_2)
        self.widget_10.setGeometry(QtCore.QRect(100, 220, 220, 101))
        self.widget_10.setStyleSheet("#widget_10 {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget_10.setObjectName("widget_10")
        self.label_dev = QtWidgets.QLabel(self.widget_10)
        self.label_dev.setGeometry(QtCore.QRect(5, 5, 211, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_dev.setFont(font)
        self.label_dev.setAlignment(QtCore.Qt.AlignCenter)
        self.label_dev.setObjectName("label_dev")
        self.lineEdit2 = QtWidgets.QLineEdit(self.widget_10)
        self.lineEdit2.setGeometry(QtCore.QRect(50, 40, 131, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.lineEdit2.setFont(font)
        self.lineEdit2.setMaxLength(1000)
        self.lineEdit2.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit2.setObjectName("lineEdit2")
        self.horizontalSlider2 = QtWidgets.QSlider(self.widget_10)
        self.horizontalSlider2.setGeometry(QtCore.QRect(50, 65, 131, 35))
        self.horizontalSlider2.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider2.setMinimum(-125)
        self.horizontalSlider2.setMaximum(125)
        self.horizontalSlider2.setPageStep(1)
        self.horizontalSlider2.setProperty("value", 0)
        self.horizontalSlider2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider2.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider2.setObjectName("horizontalSlider2")
        self.pushButton_reduce2 = QtWidgets.QPushButton(self.widget_10)
        self.pushButton_reduce2.setGeometry(QtCore.QRect(10, 65, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(30)
        self.pushButton_reduce2.setFont(font)
        self.pushButton_reduce2.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_reduce2.setObjectName("pushButton_reduce2")
        self.pushButton_plus2 = QtWidgets.QPushButton(self.widget_10)
        self.pushButton_plus2.setGeometry(QtCore.QRect(185, 65, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(25)
        self.pushButton_plus2.setFont(font)
        self.pushButton_plus2.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_plus2.setObjectName("pushButton_plus2")
        self.widget_11 = QtWidgets.QWidget(self.tab_2)
        self.widget_11.setGeometry(QtCore.QRect(810, 310, 291, 131))
        self.widget_11.setStyleSheet("#widget_11 {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget_11.setObjectName("widget_11")
        self.horizontalSlider_servoMove = QtWidgets.QSlider(self.widget_11)
        self.horizontalSlider_servoMove.setGeometry(QtCore.QRect(80, 43, 160, 35))
        self.horizontalSlider_servoMove.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider_servoMove.setMinimum(0)
        self.horizontalSlider_servoMove.setMaximum(1000)
        self.horizontalSlider_servoMove.setPageStep(1)
        self.horizontalSlider_servoMove.setProperty("value", 500)
        self.horizontalSlider_servoMove.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servoMove.setObjectName("horizontalSlider_servoMove")
        self.label_reset = QtWidgets.QLabel(self.widget_11)
        self.label_reset.setGeometry(QtCore.QRect(100, 10, 101, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_reset.setFont(font)
        self.label_reset.setAlignment(QtCore.Qt.AlignCenter)
        self.label_reset.setObjectName("label_reset")
        self.label_servoMove = QtWidgets.QLabel(self.widget_11)
        self.label_servoMove.setGeometry(QtCore.QRect(30, 45, 40, 30))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_servoMove.setFont(font)
        self.label_servoMove.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servoMove.setObjectName("label_servoMove")
        self.pushButton_resetPos = QtWidgets.QPushButton(self.widget_11)
        self.pushButton_resetPos.setGeometry(QtCore.QRect(70, 80, 171, 41))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(16)
        self.pushButton_resetPos.setFont(font)
        self.pushButton_resetPos.setStyleSheet("QPushButton{background-color: rgb(255, 165, 0);}\n"
"QPushButton:hover{background-color:  rgb(255, 210, 0);}\n"
"QPushButton{border-radius:5px;}")
        self.pushButton_resetPos.setObjectName("pushButton_resetPos")
        self.label_tips = QtWidgets.QLabel(self.tab_2)
        self.label_tips.setGeometry(QtCore.QRect(80, 30, 1041, 22))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.label_tips.setFont(font)
        self.label_tips.setAlignment(QtCore.Qt.AlignCenter)
        self.label_tips.setObjectName("label_tips")
        self.pushButton_quit2 = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_quit2.setGeometry(QtCore.QRect(820, 490, 100, 50))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(16)
        self.pushButton_quit2.setFont(font)
        self.pushButton_quit2.setStyleSheet("QPushButton{background-color: rgb(255, 165, 0);}\n"
"QPushButton:hover{background-color:  rgb(255, 210, 0);}\n"
"QPushButton{border-radius:5px;}")
        self.pushButton_quit2.setObjectName("pushButton_quit2")
        self.radioButton_zn = QtWidgets.QRadioButton(self.tab_2)
        self.radioButton_zn.setGeometry(QtCore.QRect(10, 0, 61, 27))
        self.radioButton_zn.setObjectName("radioButton_zn")
        self.radioButton_en = QtWidgets.QRadioButton(self.tab_2)
        self.radioButton_en.setGeometry(QtCore.QRect(90, 0, 81, 27))
        self.radioButton_en.setObjectName("radioButton_en")
        self.tabWidget.addTab(self.tab_2, "")

        self.retranslateUi(Form)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Servo Tool"))
        self.pushButton_read.setText(_translate("Form", "读取"))
        self.pushButton_set.setText(_translate("Form", "设置"))
        self.pushButton_default.setText(_translate("Form", "默认"))
        self.label_angle.setText(_translate("Form", "角度范围"))
        self.label_servoMin.setText(_translate("Form", "0"))
        self.label_servoMax.setText(_translate("Form", "1000"))
        self.label_vin.setText(_translate("Form", "电压范围"))
        self.label_servoMinV.setText(_translate("Form", "4.5V"))
        self.label_servoMaxV.setText(_translate("Form", "14V"))
        self.label_temp.setText(_translate("Form", "温度范围"))
        self.label_servoTemp.setText(_translate("Form", "85℃"))
        self.label_current_angle.setText(_translate("Form", "当前角度："))
        self.label_current_temp.setText(_translate("Form", "当前温度："))
        self.label_current_vin.setText(_translate("Form", "当前电压："))
        self.label_current_id.setText(_translate("Form", "当前舵机ID："))
        self.label_id.setText(_translate("Form", "舵机ID"))
        self.lineEdit1.setText(_translate("Form", "-1"))
        self.pushButton_reduce1.setText(_translate("Form", "-"))
        self.pushButton_plus1.setText(_translate("Form", "+"))
        self.label_dev.setText(_translate("Form", "舵机偏差(-125~125)"))
        self.lineEdit2.setText(_translate("Form", "0"))
        self.pushButton_reduce2.setText(_translate("Form", "-"))
        self.pushButton_plus2.setText(_translate("Form", "+"))
        self.label_reset.setText(_translate("Form", "舵机调试"))
        self.label_servoMove.setText(_translate("Form", "500"))
        self.pushButton_resetPos.setText(_translate("Form", "中位"))
        self.label_tips.setText(_translate("Form", "<html><head/><body><p><span style=\" font-weight:600; color:#ff0000;\">注意：使用下面的功能时，请确保控制器只连接了一个舵机，否则会引起冲突!</span></p></body></html>"))
        self.pushButton_quit2.setText(_translate("Form", "退出"))
        self.radioButton_zn.setText(_translate("Form", "中文"))
        self.radioButton_en.setText(_translate("Form", "English"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("Form", "舵机调试工具"))

