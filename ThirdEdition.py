from PyQt5 import QtCore, QtGui, QtWidgets
from pyqtgraph import PlotWidget
import Adafruit_BMP.BMP085 as BMP085
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QCheckBox, QWidget
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import os
from PyQt5.QtGui import*
from PyQt5.QtWidgets import*
from PyQt5.QtCore import*
import traceback, sys
import numpy as np
import Adxl345_1 as adxl
import Gyro2 as gyro2
import HMC5883L as hmcl
import RPi.GPIO as GPIO
from time import sleep
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
thruster2 = 35
thruster3 = 33
thruster1 = 32
GPIO.setup(thruster1, GPIO.OUT)
GPIO.setup(thruster2, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(thruster3, GPIO.OUT, initial = GPIO.LOW)
t1 = GPIO.PWM(thruster1,50)
t2 = GPIO.PWM(thruster2,50)
t3 = GPIO.PWM(thruster3,50)
t1.start(0)
t2.start(0)
t3.start(0)

gyro2.calibrateGyro()
compass = hmcl.hmc5883l(gauss = 0.50729, declination = (2,50))
sensor = BMP085.BMP085(1)

class Worker(QRunnable):
   def __init__(self, fn, *args, **kwargs):
      super(Worker, self).__init__()
      self.fn = fn
      self.args = args
      self.kwargs = kwargs

   @pyqtSlot()
   def run(self):
      result = self.fn(*self.args,**self.kwargs)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1366, 706)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 1366, 731))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.tabWidget.setFont(font)
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.pushButton_2 = QtWidgets.QPushButton(self.tab)
        self.pushButton_2.setGeometry(QtCore.QRect(333, 160, 350, 160))
        font = QtGui.QFont()
        font.setFamily("Sitka Small")
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setAutoFillBackground(False)
        self.pushButton_2.setStyleSheet("background-color: rgb(0, 255, 0);")
        self.pushButton_2.setAutoDefault(False)
        self.pushButton_2.setDefault(False)
        self.pushButton_2.setFlat(False)
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.tab)
        self.pushButton_3.setGeometry(QtCore.QRect(683, 160, 350, 160))
        font = QtGui.QFont()
        font.setFamily("Sitka Small")
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(self.tab)
        self.pushButton_4.setGeometry(QtCore.QRect(333, 320, 350, 160))
        font = QtGui.QFont()
        font.setFamily("Sitka Small")
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_4.setFont(font)
        self.pushButton_4.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_5 = QtWidgets.QPushButton(self.tab)
        self.pushButton_5.setGeometry(QtCore.QRect(683, 320, 350, 160))
        font = QtGui.QFont()
        font.setFamily("Sitka Small")
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_5.setFont(font)
        self.pushButton_5.setStyleSheet("background-color: rgb(255, 170, 127);")
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_6 = QtWidgets.QPushButton(self.tab)
        self.pushButton_6.setGeometry(QtCore.QRect(333, 480, 700, 160))
        font = QtGui.QFont()
        font.setFamily("Sitka Small")
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_6.setFont(font)
        self.pushButton_6.setStyleSheet("background-color: rgb(170, 85, 255);")
        self.pushButton_6.setObjectName("pushButton_6")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.tab)
        self.plainTextEdit.setGeometry(QtCore.QRect(0, 210, 331, 431))
        self.plainTextEdit.setFrameShadow(QtWidgets.QFrame.Raised)
        self.plainTextEdit.setReadOnly(True)
        self.plainTextEdit.setPlainText("")
        self.plainTextEdit.setOverwriteMode(True)
        self.plainTextEdit.setObjectName("plainTextEdit")
        font = QtGui.QFont()
        font.setPointSize(20)
        self.plainTextEdit.setFont(font)
        self.label_48 = QtWidgets.QLabel(self.tab)
        self.label_48.setGeometry(QtCore.QRect(0, 160, 331, 51))
        font = QtGui.QFont()
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.label_48.setFont(font)
        self.label_48.setStyleSheet("background-color: rgb(255, 35, 226);")
        self.label_48.setAlignment(QtCore.Qt.AlignCenter)
        self.label_48.setObjectName("label_48")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.widget = PlotWidget(self.tab_2)
        self.widget.setGeometry(QtCore.QRect(0, 0, 341, 150))
        self.widget.setObjectName("widget")
        self.widget_2 = PlotWidget(self.tab_2)
        self.widget_2.setGeometry(QtCore.QRect(341, 0, 341, 150))
        self.widget_2.setObjectName("widget_2")
        self.widget_3 = PlotWidget(self.tab_2)
        self.widget_3.setGeometry(QtCore.QRect(682, 0, 341, 150))
        self.widget_3.setObjectName("widget_3")
        self.widget_4 = PlotWidget(self.tab_2)
        self.widget_4.setGeometry(QtCore.QRect(1020, 0, 341, 150))
        self.widget_4.setObjectName("widget_4")
        self.widget_5 = PlotWidget(self.tab_2)
        self.widget_5.setGeometry(QtCore.QRect(0, 150, 341, 150))
        self.widget_5.setObjectName("widget_5")
        self.label = QtWidgets.QLabel(self.tab_2)
        self.label.setGeometry(QtCore.QRect(1100, 180, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label.setFont(font)
        self.label.setStyleSheet("")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.tab_2)
        self.label_2.setGeometry(QtCore.QRect(1100, 240, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_2.setFont(font)
        self.label_2.setStyleSheet("")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.tab_2)
        self.label_3.setGeometry(QtCore.QRect(1100, 210, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_3.setFont(font)
        self.label_3.setStyleSheet("")
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.tab_2)
        self.label_4.setGeometry(QtCore.QRect(1100, 270, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_4.setFont(font)
        self.label_4.setStyleSheet("")
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.tab_2)
        self.label_5.setGeometry(QtCore.QRect(1100, 300, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_5.setFont(font)
        self.label_5.setStyleSheet("")
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.tab_2)
        self.label_6.setGeometry(QtCore.QRect(1100, 330, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_6.setFont(font)
        self.label_6.setStyleSheet("")
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.tab_2)
        self.label_7.setGeometry(QtCore.QRect(1100, 510, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_7.setFont(font)
        self.label_7.setStyleSheet("")
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.tab_2)
        self.label_8.setGeometry(QtCore.QRect(1100, 480, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_8.setFont(font)
        self.label_8.setStyleSheet("")
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.tab_2)
        self.label_9.setGeometry(QtCore.QRect(1100, 360, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_9.setFont(font)
        self.label_9.setStyleSheet("")
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.tab_2)
        self.label_10.setGeometry(QtCore.QRect(1100, 390, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_10.setFont(font)
        self.label_10.setStyleSheet("")
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.tab_2)
        self.label_11.setGeometry(QtCore.QRect(1100, 420, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_11.setFont(font)
        self.label_11.setStyleSheet("")
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.label_12 = QtWidgets.QLabel(self.tab_2)
        self.label_12.setGeometry(QtCore.QRect(1100, 450, 41, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_12.setFont(font)
        self.label_12.setStyleSheet("")
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(self.tab_2)
        self.label_13.setGeometry(QtCore.QRect(1160, 330, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_13.setFont(font)
        self.label_13.setStyleSheet("")
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.label_14 = QtWidgets.QLabel(self.tab_2)
        self.label_14.setGeometry(QtCore.QRect(1160, 360, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_14.setFont(font)
        self.label_14.setStyleSheet("")
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.label_15 = QtWidgets.QLabel(self.tab_2)
        self.label_15.setGeometry(QtCore.QRect(1160, 270, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_15.setFont(font)
        self.label_15.setStyleSheet("")
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setObjectName("label_15")
        self.label_16 = QtWidgets.QLabel(self.tab_2)
        self.label_16.setGeometry(QtCore.QRect(1160, 510, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_16.setFont(font)
        self.label_16.setStyleSheet("")
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.label_17 = QtWidgets.QLabel(self.tab_2)
        self.label_17.setGeometry(QtCore.QRect(1160, 480, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_17.setFont(font)
        self.label_17.setStyleSheet("")
        self.label_17.setAlignment(QtCore.Qt.AlignCenter)
        self.label_17.setObjectName("label_17")
        self.label_18 = QtWidgets.QLabel(self.tab_2)
        self.label_18.setGeometry(QtCore.QRect(1160, 210, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_18.setFont(font)
        self.label_18.setStyleSheet("")
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setObjectName("label_18")
        self.label_19 = QtWidgets.QLabel(self.tab_2)
        self.label_19.setGeometry(QtCore.QRect(1160, 240, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_19.setFont(font)
        self.label_19.setStyleSheet("")
        self.label_19.setAlignment(QtCore.Qt.AlignCenter)
        self.label_19.setObjectName("label_19")
        self.label_20 = QtWidgets.QLabel(self.tab_2)
        self.label_20.setGeometry(QtCore.QRect(1160, 300, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_20.setFont(font)
        self.label_20.setStyleSheet("")
        self.label_20.setAlignment(QtCore.Qt.AlignCenter)
        self.label_20.setObjectName("label_20")
        self.label_21 = QtWidgets.QLabel(self.tab_2)
        self.label_21.setGeometry(QtCore.QRect(1160, 420, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_21.setFont(font)
        self.label_21.setStyleSheet("")
        self.label_21.setAlignment(QtCore.Qt.AlignCenter)
        self.label_21.setObjectName("label_21")
        self.label_22 = QtWidgets.QLabel(self.tab_2)
        self.label_22.setGeometry(QtCore.QRect(1160, 450, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_22.setFont(font)
        self.label_22.setStyleSheet("")
        self.label_22.setAlignment(QtCore.Qt.AlignCenter)
        self.label_22.setObjectName("label_22")
        self.label_23 = QtWidgets.QLabel(self.tab_2)
        self.label_23.setGeometry(QtCore.QRect(1160, 180, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_23.setFont(font)
        self.label_23.setStyleSheet("")
        self.label_23.setAlignment(QtCore.Qt.AlignCenter)
        self.label_23.setObjectName("label_23")
        self.label_24 = QtWidgets.QLabel(self.tab_2)
        self.label_24.setGeometry(QtCore.QRect(1160, 390, 90, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_24.setFont(font)
        self.label_24.setStyleSheet("")
        self.label_24.setAlignment(QtCore.Qt.AlignCenter)
        self.label_24.setObjectName("label_24")
        self.label_25 = QtWidgets.QLabel(self.tab_2)
        self.label_25.setGeometry(QtCore.QRect(1260, 330, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_25.setFont(font)
        self.label_25.setStyleSheet("")
        self.label_25.setAlignment(QtCore.Qt.AlignCenter)
        self.label_25.setObjectName("label_25")
        self.label_26 = QtWidgets.QLabel(self.tab_2)
        self.label_26.setGeometry(QtCore.QRect(1260, 360, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_26.setFont(font)
        self.label_26.setStyleSheet("")
        self.label_26.setAlignment(QtCore.Qt.AlignCenter)
        self.label_26.setObjectName("label_26")
        self.label_27 = QtWidgets.QLabel(self.tab_2)
        self.label_27.setGeometry(QtCore.QRect(1260, 270, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_27.setFont(font)
        self.label_27.setStyleSheet("")
        self.label_27.setAlignment(QtCore.Qt.AlignCenter)
        self.label_27.setObjectName("label_27")
        self.label_28 = QtWidgets.QLabel(self.tab_2)
        self.label_28.setGeometry(QtCore.QRect(1260, 510, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_28.setFont(font)
        self.label_28.setStyleSheet("")
        self.label_28.setAlignment(QtCore.Qt.AlignCenter)
        self.label_28.setObjectName("label_28")
        self.label_29 = QtWidgets.QLabel(self.tab_2)
        self.label_29.setGeometry(QtCore.QRect(1260, 480, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_29.setFont(font)
        self.label_29.setStyleSheet("")
        self.label_29.setAlignment(QtCore.Qt.AlignCenter)
        self.label_29.setObjectName("label_29")
        self.label_30 = QtWidgets.QLabel(self.tab_2)
        self.label_30.setGeometry(QtCore.QRect(1260, 210, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_30.setFont(font)
        self.label_30.setStyleSheet("")
        self.label_30.setAlignment(QtCore.Qt.AlignCenter)
        self.label_30.setObjectName("label_30")
        self.label_31 = QtWidgets.QLabel(self.tab_2)
        self.label_31.setGeometry(QtCore.QRect(1260, 240, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_31.setFont(font)
        self.label_31.setStyleSheet("")
        self.label_31.setAlignment(QtCore.Qt.AlignCenter)
        self.label_31.setObjectName("label_31")
        self.label_32 = QtWidgets.QLabel(self.tab_2)
        self.label_32.setGeometry(QtCore.QRect(1260, 300, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_32.setFont(font)
        self.label_32.setStyleSheet("")
        self.label_32.setAlignment(QtCore.Qt.AlignCenter)
        self.label_32.setObjectName("label_32")
        self.label_33 = QtWidgets.QLabel(self.tab_2)
        self.label_33.setGeometry(QtCore.QRect(1260, 420, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_33.setFont(font)
        self.label_33.setStyleSheet("")
        self.label_33.setAlignment(QtCore.Qt.AlignCenter)
        self.label_33.setObjectName("label_33")
        self.label_34 = QtWidgets.QLabel(self.tab_2)
        self.label_34.setGeometry(QtCore.QRect(1260, 450, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_34.setFont(font)
        self.label_34.setStyleSheet("")
        self.label_34.setAlignment(QtCore.Qt.AlignCenter)
        self.label_34.setObjectName("label_34")
        self.label_35 = QtWidgets.QLabel(self.tab_2)
        self.label_35.setGeometry(QtCore.QRect(1260, 180, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_35.setFont(font)
        self.label_35.setStyleSheet("")
        self.label_35.setAlignment(QtCore.Qt.AlignCenter)
        self.label_35.setObjectName("label_35")
        self.label_36 = QtWidgets.QLabel(self.tab_2)
        self.label_36.setGeometry(QtCore.QRect(1260, 390, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_36.setFont(font)
        self.label_36.setStyleSheet("")
        self.label_36.setAlignment(QtCore.Qt.AlignCenter)
        self.label_36.setObjectName("label_36")
        self.label_37 = QtWidgets.QLabel(self.tab_2)
        self.label_37.setGeometry(QtCore.QRect(1110, 160, 201, 21))
        font = QtGui.QFont()
        font.setPointSize(13)
        self.label_37.setFont(font)
        self.label_37.setStyleSheet("background-color: rgb(234, 255, 175);\n"
"")
        self.label_37.setFrameShadow(QtWidgets.QFrame.Raised)
        self.label_37.setAlignment(QtCore.Qt.AlignCenter)
        self.label_37.setObjectName("label_37")
        ############Horizontal Slider##################################
        self.horizontalSlider = QtWidgets.QSlider(self.tab_2)
        self.horizontalSlider.setGeometry(QtCore.QRect(10, 330, 271, 31))
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.horizontalSlider.setMaximum(100)
        self.horizontalSlider.setMinimum(0)
        self.horizontalSlider_2 = QtWidgets.QSlider(self.tab_2)
        self.horizontalSlider_2.setGeometry(QtCore.QRect(10, 380, 271, 31))
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.horizontalSlider_2.setMaximum(100)
        self.horizontalSlider_2.setMinimum(0)
        self.horizontalSlider_3 = QtWidgets.QSlider(self.tab_2)
        self.horizontalSlider_3.setGeometry(QtCore.QRect(10, 430, 271, 31))
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.horizontalSlider_3.setMaximum(100)
        self.horizontalSlider_3.setMinimum(0)
        self.horizontalSlider_4 = QtWidgets.QSlider(self.tab_2)
        self.horizontalSlider_4.setGeometry(QtCore.QRect(10, 480, 271, 31))
        self.horizontalSlider_4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_4.setObjectName("horizontalSlider_4")
        self.horizontalSlider_4.setMaximum(100)
        self.horizontalSlider_4.setMinimum(0)
        ##################################################################
        self.label_38 = QtWidgets.QLabel(self.tab_2)
        self.label_38.setGeometry(QtCore.QRect(150, 310, 81, 20))
        self.label_38.setAlignment(QtCore.Qt.AlignCenter)
        self.label_38.setObjectName("label_38")
        self.label_39 = QtWidgets.QLabel(self.tab_2)
        self.label_39.setGeometry(QtCore.QRect(150, 360, 81, 20))
        self.label_39.setAlignment(QtCore.Qt.AlignCenter)
        self.label_39.setObjectName("label_39")
        self.label_40 = QtWidgets.QLabel(self.tab_2)
        self.label_40.setGeometry(QtCore.QRect(150, 410, 81, 20))
        self.label_40.setAlignment(QtCore.Qt.AlignCenter)
        self.label_40.setObjectName("label_40")
        self.label_41 = QtWidgets.QLabel(self.tab_2)
        self.label_41.setGeometry(QtCore.QRect(150, 460, 81, 20))
        self.label_41.setAlignment(QtCore.Qt.AlignCenter)
        self.label_41.setObjectName("label_41")
        self.label_42 = QtWidgets.QLabel(self.tab_2)
        self.label_42.setGeometry(QtCore.QRect(60, 310, 81, 20))
        self.label_42.setAlignment(QtCore.Qt.AlignCenter)
        self.label_42.setObjectName("label_42")
        self.label_43 = QtWidgets.QLabel(self.tab_2)
        self.label_43.setGeometry(QtCore.QRect(60, 360, 81, 20))
        self.label_43.setAlignment(QtCore.Qt.AlignCenter)
        self.label_43.setObjectName("label_43")
        self.label_44 = QtWidgets.QLabel(self.tab_2)
        self.label_44.setGeometry(QtCore.QRect(60, 410, 81, 20))
        self.label_44.setAlignment(QtCore.Qt.AlignCenter)
        self.label_44.setObjectName("label_44")
        self.label_45 = QtWidgets.QLabel(self.tab_2)
        self.label_45.setGeometry(QtCore.QRect(60, 460, 81, 20))
        self.label_45.setAlignment(QtCore.Qt.AlignCenter)
        self.label_45.setObjectName("label_45")
        ######################Vertical Slider###########################
        self.verticalSlider = QtWidgets.QSlider(self.tab_2)
        self.verticalSlider.setGeometry(QtCore.QRect(310, 310, 31, 261))
        self.verticalSlider.setOrientation(QtCore.Qt.Vertical)
        self.verticalSlider.setObjectName("verticalSlider")
        self.verticalSlider.setMaximum(100)
        self.verticalSlider.setMinimum(0)
        ################################################################
        self.label_46 = QtWidgets.QLabel(self.tab_2)
        self.label_46.setGeometry(QtCore.QRect(300, 580, 47, 13))
        self.label_46.setAlignment(QtCore.Qt.AlignCenter)
        self.label_46.setObjectName("label_46")
        self.label_47 = QtWidgets.QLabel(self.tab_2)
        self.label_47.setGeometry(QtCore.QRect(250, 410, 61, 20))
        self.label_47.setAlignment(QtCore.Qt.AlignCenter)
        self.label_47.setObjectName("label_47")
        self.widget_6 = QtWidgets.QWidget(self.tab_2)
        self.widget_6.setGeometry(QtCore.QRect(350, 170, 721, 441))
        self.widget_6.setObjectName("widget_6")
         #################Steaming First###################################
        self.checkBox_2 = QtWidgets.QCheckBox(self.tab_2)
        self.checkBox_2.setGeometry(QtCore.QRect(350, 150, 80, 17))
        self.checkBox_2.setObjectName("checkBox_2")
        ###################################################################
        self.pushButton_7 = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_7.setGeometry(QtCore.QRect(10, 560, 140, 20))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.pushButton_7.setFont(font)
        self.pushButton_7.setAutoDefault(False)
        self.pushButton_7.setDefault(False)
        self.pushButton_7.setFlat(False)
        self.pushButton_7.setObjectName("pushButton_7")
        #self.pushButton_7.clicked.connect(self.RESET)

        self.pushButton_8 = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_8.setGeometry(QtCore.QRect(150, 560, 140, 20))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.pushButton_8.setFont(font)
        self.pushButton_8.setAutoDefault(False)
        self.pushButton_8.setDefault(False)
        self.pushButton_8.setFlat(False)
        self.pushButton_8.setObjectName("pushButton_8")
        #self.pushButton_8.clicked.connect(self.RESET)
        ###################################################################
        self.label_49 = QtWidgets.QLabel(self.tab_2)
        self.label_49.setGeometry(QtCore.QRect(1140, 530, 161, 101))
        self.label_49.setText("")
        self.label_49.setPixmap(QtGui.QPixmap("Capture.PNG"))
        self.label_49.setObjectName("label_49")
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.widget_7 = QtWidgets.QWidget(self.tab_3)
        self.widget_7.setGeometry(QtCore.QRect(19, 19, 1311, 661))
        self.widget_7.setObjectName("widget_7")
        self.checkBox_3 = QtWidgets.QCheckBox(self.tab_3)
        self.checkBox_3.setGeometry(QtCore.QRect(20, 0, 80, 17))
        self.checkBox_3.setObjectName("checkBox_3")
        self.tabWidget.addTab(self.tab_3, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1366, 21))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuKey_Map = QtWidgets.QMenu(self.menuFile)
        self.menuKey_Map.setObjectName("menuKey_Map")
        self.menuEdit = QtWidgets.QMenu(self.menubar)
        self.menuEdit.setObjectName("menuEdit")
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionForward = QtWidgets.QAction(MainWindow)
        self.actionForward.setObjectName("actionForward")
        self.actionBackward = QtWidgets.QAction(MainWindow)
        self.actionBackward.setObjectName("actionBackward")
        self.actionYaw_Left = QtWidgets.QAction(MainWindow)
        self.actionYaw_Left.setObjectName("actionYaw_Left")
        self.actionYaw_Right = QtWidgets.QAction(MainWindow)
        self.actionYaw_Right.setObjectName("actionYaw_Right")
        self.actionPitch_Up = QtWidgets.QAction(MainWindow)
        self.actionPitch_Up.setObjectName("actionPitch_Up")
        self.actionPitch_Down = QtWidgets.QAction(MainWindow)
        self.actionPitch_Down.setObjectName("actionPitch_Down")
        self.actionRoll_Left = QtWidgets.QAction(MainWindow)
        self.actionRoll_Left.setObjectName("actionRoll_Left")
        self.actionRoll_Right = QtWidgets.QAction(MainWindow)
        self.actionRoll_Right.setObjectName("actionRoll_Right")
        self.actionHeave_Up = QtWidgets.QAction(MainWindow)
        self.actionHeave_Up.setObjectName("actionHeave_Up")
        self.actionHeave_Down = QtWidgets.QAction(MainWindow)
        self.actionHeave_Down.setObjectName("actionHeave_Down")
        self.actionOpen_Data_Log_File = QtWidgets.QAction(MainWindow)
        self.actionOpen_Data_Log_File.setObjectName("actionOpen_Data_Log_File")
        self.actionDocummentation = QtWidgets.QAction(MainWindow)
        self.actionDocummentation.setObjectName("actionDocummentation")
        self.actionAbout = QtWidgets.QAction(MainWindow)
        self.actionAbout.setObjectName("actionAbout")
        self.actionAbout_Program = QtWidgets.QAction(MainWindow)
        self.actionAbout_Program.setObjectName("actionAbout_Program")
        self.actionQuit = QtWidgets.QAction(MainWindow)
        self.actionQuit.setObjectName("actionQuit")
        self.menuKey_Map.addAction(self.actionForward)
        self.menuKey_Map.addAction(self.actionBackward)
        self.menuKey_Map.addAction(self.actionYaw_Left)
        self.menuKey_Map.addAction(self.actionYaw_Right)
        self.menuKey_Map.addAction(self.actionPitch_Up)
        self.menuKey_Map.addAction(self.actionPitch_Down)
        self.menuKey_Map.addAction(self.actionRoll_Left)
        self.menuKey_Map.addAction(self.actionRoll_Right)
        self.menuKey_Map.addAction(self.actionHeave_Up)
        self.menuKey_Map.addAction(self.actionHeave_Down)
        self.menuFile.addAction(self.menuKey_Map.menuAction())
        self.menuFile.addAction(self.actionQuit)
        self.menuEdit.addAction(self.actionOpen_Data_Log_File)
        self.menuHelp.addAction(self.actionDocummentation)
        self.menuHelp.addAction(self.actionAbout)
        self.menuHelp.addAction(self.actionAbout_Program)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_2.setText(_translate("MainWindow", "Start Operation"))
        self.pushButton_3.setText(_translate("MainWindow", "Stop Operation"))
        self.pushButton_4.setText(_translate("MainWindow", "Open Loop "))
        self.pushButton_5.setText(_translate("MainWindow", "Closed Loop"))
        self.pushButton_6.setText(_translate("MainWindow", "Check System"))
        self.pushButton_7.setText(_translate("MainWindow", "Reset Plots"))
        self.pushButton_8.setText(_translate("MainWindow", "Start Data Logging"))
        self.label_48.setText(_translate("MainWindow", "Status"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Tab 1"))
        self.label.setText(_translate("MainWindow", "gx"))
        self.label_2.setText(_translate("MainWindow", "gz"))
        self.label_3.setText(_translate("MainWindow", "gy"))
        self.label_4.setText(_translate("MainWindow", "wx"))
        self.label_5.setText(_translate("MainWindow", "wy"))
        self.label_6.setText(_translate("MainWindow", "wz"))
        self.label_7.setText(_translate("MainWindow", "P"))
        self.label_8.setText(_translate("MainWindow", "T"))
        self.label_9.setText(_translate("MainWindow", "Pitch"))
        self.label_10.setText(_translate("MainWindow", "Roll"))
        self.label_11.setText(_translate("MainWindow", "Yaw"))
        self.label_12.setText(_translate("MainWindow", "Speed"))
        self.label_13.setText(_translate("MainWindow", "0"))
        self.label_14.setText(_translate("MainWindow", "0"))
        self.label_15.setText(_translate("MainWindow", "0"))
        self.label_16.setText(_translate("MainWindow", "0"))
        self.label_17.setText(_translate("MainWindow", "0"))
        self.label_18.setText(_translate("MainWindow", "0"))
        self.label_19.setText(_translate("MainWindow", "0"))
        self.label_20.setText(_translate("MainWindow", "0"))
        self.label_21.setText(_translate("MainWindow", "0"))
        self.label_22.setText(_translate("MainWindow", "0"))
        self.label_23.setText(_translate("MainWindow", "0"))
        self.label_24.setText(_translate("MainWindow", "0"))
        self.label_25.setText(_translate("MainWindow", "deg/sec"))
        self.label_26.setText(_translate("MainWindow", "deg"))
        self.label_27.setText(_translate("MainWindow", "deg/sec"))
        self.label_28.setText(_translate("MainWindow", "KPa"))
        self.label_29.setText(_translate("MainWindow", "Deg C"))
        self.label_30.setText(_translate("MainWindow", "m/sec2"))
        self.label_31.setText(_translate("MainWindow", "m/sec2"))
        self.label_32.setText(_translate("MainWindow", "deg/sec"))
        self.label_33.setText(_translate("MainWindow", "deg"))
        self.label_34.setText(_translate("MainWindow", "m/sec"))
        self.label_35.setText(_translate("MainWindow", "m/sec2"))
        self.label_36.setText(_translate("MainWindow", "deg"))
        self.label_37.setText(_translate("MainWindow", "Raw Telemetry Data"))
        self.label_38.setText(_translate("MainWindow", "0"))
        self.label_39.setText(_translate("MainWindow", "0"))
        self.label_40.setText(_translate("MainWindow", "0"))
        self.label_41.setText(_translate("MainWindow", "0"))
        self.label_42.setText(_translate("MainWindow", "Yaw"))
        self.label_43.setText(_translate("MainWindow", "Pitch"))
        self.label_44.setText(_translate("MainWindow", "Roll"))
        self.label_45.setText(_translate("MainWindow", "Heave"))
        self.label_46.setText(_translate("MainWindow", "Surge"))
        self.label_47.setText(_translate("MainWindow", "0"))
        self.checkBox_2.setText(_translate("MainWindow", "Streaming"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Tab 2"))
        self.checkBox_3.setText(_translate("MainWindow", "Streaming"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("MainWindow", "Tab 3"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuKey_Map.setTitle(_translate("MainWindow", "Key Map"))
        self.menuEdit.setTitle(_translate("MainWindow", "Edit"))
        self.menuHelp.setTitle(_translate("MainWindow", "Help"))
        self.actionForward.setText(_translate("MainWindow", "Forward"))
        self.actionForward.setShortcut(_translate("MainWindow", "Up"))
        self.actionBackward.setText(_translate("MainWindow", "Backward"))
        self.actionBackward.setShortcut(_translate("MainWindow", "Down"))
        self.actionYaw_Left.setText(_translate("MainWindow", "Yaw Left"))
        self.actionYaw_Left.setShortcut(_translate("MainWindow", "Left"))
        self.actionYaw_Right.setText(_translate("MainWindow", "Yaw Right"))
        self.actionYaw_Right.setShortcut(_translate("MainWindow", "Right"))
        self.actionPitch_Up.setText(_translate("MainWindow", "Pitch Up"))
        self.actionPitch_Up.setShortcut(_translate("MainWindow", "Q"))
        self.actionPitch_Down.setText(_translate("MainWindow", "Pitch Down"))
        self.actionPitch_Down.setShortcut(_translate("MainWindow", "E"))
        self.actionRoll_Left.setText(_translate("MainWindow", "Roll Left"))
        self.actionRoll_Left.setShortcut(_translate("MainWindow", "A"))
        self.actionRoll_Right.setText(_translate("MainWindow", "Roll Right"))
        self.actionRoll_Right.setShortcut(_translate("MainWindow", "D"))
        self.actionHeave_Up.setText(_translate("MainWindow", "Heave Up"))
        self.actionHeave_Up.setShortcut(_translate("MainWindow", "W"))
        self.actionHeave_Down.setText(_translate("MainWindow", "Heave Down"))
        self.actionHeave_Down.setShortcut(_translate("MainWindow", "S"))
        self.actionOpen_Data_Log_File.setText(_translate("MainWindow", "Open Data Log File"))
        self.actionDocummentation.setText(_translate("MainWindow", "Docummentation"))
        self.actionAbout.setText(_translate("MainWindow", "About ROV"))
        self.actionAbout_Program.setText(_translate("MainWindow", "About Program"))
        self.actionQuit.setText(_translate("MainWindow", "Quit"))


#######################################################################
###################Commands go here####################################
        self.threadpool = QThreadPool()
        self.pushButton_2.clicked.connect(self.StartButton)
        self.pushButton_3.clicked.connect(self.StopButton)
        self.pushButton_4.clicked.connect(self.OPEN)
        self.pushButton_5.clicked.connect(self.CLOSED)
        self.pushButton_8.clicked.connect(self.logData)
        # self.verticalSlider.valueChanged[int].connect(self.Slider_1ValueChanged)
        # self.horizontalSlider.valueChanged[int].connect(self.Slider_2ValueChanged)
        # self.horizontalSlider_2.valueChanged[int].connect(self.Slider_2ValueChanged)
        # self.horizontalSlider_3.valueChanged[int].connect(self.Slider_3ValueChanged)
        # self.horizontalSlider_4.valueChanged[int].connect(self.Slider_4ValueChanged)
        self.actionForward.triggered.connect(self.FORWARD)
        self.actionBackward.triggered.connect(self.BACKWARD)
        self.actionYaw_Left.triggered.connect(self.YAWLEFT)
        self.actionYaw_Right.triggered.connect(self.YAWRIGHT)
        self.actionPitch_Up.triggered.connect(self.PITCHUP)
        self.actionPitch_Down.triggered.connect(self.PITCHDOWN)
        self.actionRoll_Right.triggered.connect(self.ROLLCW)
        self.actionRoll_Left.triggered.connect(self.ROLLCCW)
        self.actionHeave_Up.triggered.connect(self.HEAVEUP)
        self.actionHeave_Down.triggered.connect(self.HEAVEDOWN)
#######################################################################
###################Declaration go here#################################

        self.Lstartbutton = []
        self.OpenLoop     = []
        self.ClosedLoop   = []
        self.pressure     = []
        self.temperature  = []
        self.firstRun     = []
        self.firstRun1    = []
        self.firstRun2    = []
        self.firstRun3    = []
        self.firstRun4    = []
        self.firstRun5    = []
        self.fileName     = ''
        self.fileName1    = ''
        self.t            = []
        self.t1           = []
        self.DataPoints   = 20
        self.roll         = []
        self.rollDrift    = []
        self.pitch        = []
        self.pitchDrift   = []
        self.rollAccel    = []
        self.pitchAccel   = []
        self.sliderVal1   = [0]
        self.sliderVal2   = [0]
        self.sliderVal3   = [0]
        self.sliderVal4   = [0]
        self.sliderVal5   = [0]
        self.DLogButton   = []
#######################################################################
###################Plots customization go here#########################
        self.widget.setTitle('Time vs Temperature')
        self.widget_2.setTitle('Time vs Pressure')
        self.widget_3.setTitle('Time vs Roll Angle')
        self.widget_4.setTitle('Time vs Pitch Angle')
        pen  = pg.mkPen(color = (255, 0, 0), width = 2)
        pen1 = pg.mkPen(color = (0, 255, 0), width = 2)
        pen2 = pg.mkPen(color = (0, 0, 255), width = 2)
        font2=QtGui.QFont()
        font2.setPixelSize(11)
        self.widget.setYRange(0,50)
        self.widget_2.setYRange(96000,98000)
#######################################################################
###################Plots go here#######################################
        self.plot_1 = self.widget  .plot(self.t1,   self.pressure,pen=pen,fillLevel = 1,fillBrush = (255,255,255,50))
        self.plot_2 = self.widget_2.plot(self.t1,self.temperature,pen=pen,fillLevel = 1,fillBrush = (255,255,255,50))
        self.plot_3 = self.widget_3.plot(self.t,self.roll,pen=pen)
        self.plot_4 = self.widget_3.plot(self.t,self.rollDrift,pen=pen1)
        self.plot_5 = self.widget_3.plot(self.t,self.rollAccel,pen=pen2)
        self.plot_6 = self.widget_4.plot(self.t,self.pitch,pen=pen)
        self.plot_7 = self.widget_4.plot(self.t,self.pitchDrift,pen=pen1)
        self.plot_8 = self.widget_4.plot(self.t,self.pitchAccel,pen=pen2)

#######################################################################
###################Further customization go here#######################
        yticks = np.linspace(1,51,6)
        ticks = self.widget.getAxis('left')
        ticks.setTicks([[(p,str(int(p))) for p in yticks]])
#######################################################################
###################Qtimers go here#####################################
        self.timer_1 = QtCore.QTimer()
        self.timer_1.setInterval(1)
        self.timer_1.timeout.connect(self.getValues)
        self.timer_1.start()

        self.timer_2 = QtCore.QTimer()
        self.timer_2.setInterval(100)
        self.timer_2.timeout.connect(self.thread_2)
        self.timer_2.start()

        self.timer_3 = QtCore.QTimer()
        self.timer_3.setInterval(100)
        self.timer_3.timeout.connect(self.thread_3)
        self.timer_3.start()
        ##
        self.timer_4 = QtCore.QTimer()
        self.timer_4.setInterval(100)
        self.timer_4.timeout.connect(self.thread_4)
        self.timer_4.start()

        self.timer_5 = QtCore.QTimer()
        self.timer_5.setInterval(100)
        self.timer_5.timeout.connect(self.thread_5)
        self.timer_5.start()
##
        self.timer_6 = QtCore.QTimer()
        self.timer_6.setInterval(100)
        self.timer_6.timeout.connect(self.updateLabels)
        self.timer_6.start()
##
        self.timer_7 = QtCore.QTimer()
        self.timer_7.setInterval(10)
        self.timer_7.timeout.connect(self.updateThruster1)
        self.timer_7.start()
###################################################################################################
###################Function go here################################################################
###################################################################################################

    def RESET(self):
        self.plot_1.clear()
        self.plot_2.clear()
        self.plot_3.clear()
        self.plot_4.clear()
        self.plot_5.clear()
        self.plot_6.clear()
        self.plot_7.clear()
        self.plot_8.clear()

    def StartButton(self):
        if self.ClosedLoop or self.OpenLoop:
            self.Lstartbutton.append(1)
            self.pushButton_2.setEnabled(False)
            self.pushButton_3.setEnabled(True)
            self.plainTextEdit.clear()
            self.plainTextEdit.insertPlainText('Operation Started')
        else:
            self.plainTextEdit.clear()
            self.plainTextEdit.insertPlainText('Please select Open Loop/Closed Loop')

    def StopButton(self):
        self.Lstartbutton = []
        self.pushButton_2.setEnabled(True)
        self.pushButton_3.setEnabled(False)
        self.plainTextEdit.clear()
        self.plainTextEdit.insertPlainText('Operation Stopped')

    def OPEN(self):
        self.OpenLoop.append(1)
        self.plainTextEdit.clear()
        self.plainTextEdit.insertPlainText('You are operating ROV in Open Loop')
        self.pushButton_4.setEnabled(False)
        self.pushButton_5.setEnabled(True)
        self.ClosedLoop = []

    def CLOSED(self):
        self.ClosedLoop.append(1)
        self.plainTextEdit.clear()
        self.plainTextEdit.insertPlainText('You are operating ROV in Closed Loop')
        self.pushButton_5.setEnabled(False)
        self.pushButton_4.setEnabled(True)
        self.OpenLoop = []

    def getValues(self):
       if self.Lstartbutton:
          if len(self.firstRun) <= self.DataPoints:
             self.firstRun.append(1)
             self.t = np.arange(1,len(self.firstRun)+1)
             gyro2.updateGyroValues()
             gyro2.updateHeadings()
             xi,yi,xii,yii,zii,s1,s2 = gyro2.printHeadings()
             xiii,yiii,ziii = compass.axes()
             self.roll.append(xii)
             self.rollDrift.append(s1)
             self.pitch.append(yii)
             self.pitchDrift.append(s2)
             self.rollAccel.append(xi)
             self.pitchAccel.append(yi)
          else:
             if not self.firstRun1:
                self.t = list(self.t)
                self.firstRun1.append(1)
                self.t = self.t[1:]
                self.t.append(self.t[-1]+1)
                gyro2.updateGyroValues()
                gyro2.updateHeadings()
                xi,yi,xii,yii,zii,s1,s2 = gyro2.printHeadings()
                xiii,yiii,ziii = compass.axes()
                self.roll = self.roll[1:]
                self.roll.append(xii)
                self.rollDrift = self.rollDrift[1:]
                self.rollDrift.append(s1)

             self.t = list(self.t)
             self.firstRun1.append(1)
             self.t = self.t[1:]
             self.t.append(self.t[-1]+1)
             gyro2.updateGyroValues()
             gyro2.updateHeadings()
             xi,yi,xii,yii,zii,s1,s2 = gyro2.printHeadings()
             xiii,yiii,ziii = compass.axes()
             self.roll = self.roll[1:]
             self.pitch = self.pitch[1:]
             self.rollDrift = self.rollDrift[1:]
             self.pitchDrift = self.pitchDrift[1:]
             self.rollAccel = self.rollAccel[1:]
             self.pitchAccel = self.pitchAccel[1:]
             self.roll.append(xii)
             self.rollDrift.append(s1)
             self.pitch.append(yii)
             self.pitchDrift.append(s2)
             self.rollAccel.append(xi)
             self.pitchAccel.append(yi)

    def getValues2(self):
       if self.Lstartbutton:
          if len(self.firstRun3) <= self.DataPoints:
             self.firstRun3.append(1)
             temperature    = sensor.read_temperature()
             pressure       = sensor.read_pressure()
             self.temperature.append(temperature)
             self.pressure.append(pressure)
             self.t1 = np.arange(1,len(self.temperature)+1)
          else:
             if not self.firstRun4:
                self.t1 = list(self.t)
                self.firstRun4.append(1)
                self.t1 = self.t1[1:]
                self.t1.append(self.t1[-1]+1)
                temperature = sensor.read_temperature()
                pressure    = sensor.read_pressure()
                self.temperature = self.temperature[1:]
                self.pressure    = self.pressure[1:]
                self.pressure.append(pressure)
                self.temperature.append(temperature)

             self.t1 = self.t1[1:]
             self.t1.append(self.t1[-1]+1)
             temperature = sensor.read_temperature()
             pressure    = sensor.read_pressure()
             self.temperature = self.temperature[1:]
             self.pressure    = self.pressure[1:]
             self.pressure.append(pressure)
             self.temperature.append(temperature)

    def logData(self):
       if self.Lstartbutton:
          if not self.DLogButton:
             os.system('python3 /home/pi/Qt/FinalROV/Gyro3.py &')
             self.pushButton_8.setText("Stop Data Logging")
             self.DLogButton.append(1)
             self.plainTextEdit.clear()
             self.plainTextEdit.insertPlainText('Data logging started')
          else:
             os.system('sudo pkill -f Gyro3.py')
             self.pushButton_8.setText("Start Data Logging")
             self.plainTextEdit.clear()
             self.plainTextEdit.insertPlainText('Data logging stopped')
             self.DLogButton = []
       else:
          self.plainTextEdit.clear()
          self.plainTextEdit.insertPlainText('Please Start the operation')

    def UpdatePlot_1(self):
       if self.Lstartbutton:
          self.plot_1.setData(self.t1,self.temperature)
          self.plot_2.setData(self.t1,self.pressure)


    def UpdatePlot_2(self):
      if self.Lstartbutton:
          self.plot_3.setData(self.t,self.roll)
          self.plot_4.setData(self.t,self.rollDrift)
          self.plot_5.setData(self.t,self.rollAccel)

    def UpdatePlot_3(self):
      if self.Lstartbutton:
          self.plot_6.setData(self.t,self.pitch)
          self.plot_7.setData(self.t,self.pitchDrift)
          self.plot_8.setData(self.t,self.pitchAccel)

    def updateLabels(self):
      if self.Lstartbutton:
         self.label_47.setText(str(self.sliderVal1[0]))
         self.label_38.setText(str(self.sliderVal2[0]))
         self.label_39.setText(str(self.sliderVal3[0]))
         self.label_40.setText(str(self.sliderVal4[0]))
         self.label_41.setText(str(self.sliderVal5[0]))


    def FORWARD(self):
       if self.Lstartbutton:
          if self.sliderVal1[0] != 100:
             self.sliderVal1[0] += 1
             self.verticalSlider.setValue(self.sliderVal1[0]+1)

    def updateThruster1(self):
       val = self.verticalSlider.value()
       t1.ChangeDutyCycle(val)

    def BACKWARD(self):
       if self.Lstartbutton:
          if self.sliderVal1[0] != 0:
             self.sliderVal1[0] -= 1
             self.verticalSlider.setValue(self.sliderVal1[0]-1)

    def YAWRIGHT(self):
       if self.Lstartbutton:
          if self.sliderVal2[0] != 100:
             self.sliderVal2[0] += 1
             self.horizontalSlider.setValue(self.sliderVal2[0]+1)

    def YAWLEFT(self):
       if self.Lstartbutton:
          if self.sliderVal2[0] != 0:
             self.sliderVal2[0] -= 1
             self.horizontalSlider.setValue(self.sliderVal2[0]-1)

    def PITCHDOWN(self):
       if self.Lstartbutton:
          if self.sliderVal3[0] != 100:
             self.sliderVal3[0] += 1
             self.horizontalSlider_2.setValue(self.sliderVal3[0]+1)

    def PITCHUP(self):
       if self.Lstartbutton:
          if self.sliderVal3[0] != 0:
             self.sliderVal3[0] -= 1
             self.horizontalSlider_2.setValue(self.sliderVal3[0]-1)

    def ROLLCW(self):
       if self.Lstartbutton:
          if self.sliderVal4[0] != 100:
             self.sliderVal4[0] += 1
             self.horizontalSlider_3.setValue(self.sliderVal4[0]+1)

    def ROLLCCW(self):
       if self.Lstartbutton:
          if self.sliderVal4[0] != 0:
             self.sliderVal4[0] -= 1
             self.horizontalSlider_3.setValue(self.sliderVal4[0]-1)

    def HEAVEUP(self):
       if self.Lstartbutton:
          if self.sliderVal5[0] != 100:
             self.sliderVal5[0] += 1
             self.horizontalSlider_4.setValue(self.sliderVal5[0]+1)

    def HEAVEDOWN(self):
       if self.Lstartbutton:
          if self.sliderVal5[0] != 0:
             self.sliderVal5[0] -= 1
             self.horizontalSlider_4.setValue(self.sliderVal5[0]-1)

    def thread_2(self):
      worker = Worker(self.getValues2)
      self.threadpool.start(worker)

    def thread_3(self):
      worker = Worker(self.UpdatePlot_2)
      self.threadpool.start(worker)

    def thread_4(self):
      worker = Worker(self.UpdatePlot_3)
      self.threadpool.start(worker)

    def thread_5(self):
      if len(self.t1) == len(self.temperature) == len(self.pressure):
         worker = Worker(self.UpdatePlot_1)
         self.threadpool.start(worker)
   

    def backupStopper(self):
      if self.DLogButton:
         os.system('sudo pkill -f Gyro3.py')
         print('You left data logging running, I stopped it for you')
      else:
         pass
      
    def cleanup(self):
      GPIO.cleanup()  # this switch off all the pins on Raspberry pi
       

    def appExec(self):
      app.exec_()
      self.cleanup()
      self.backupStopper()


if __name__ == "__main__":
   import sys
   app = QtWidgets.QApplication(sys.argv)
   MainWindow = QtWidgets.QMainWindow()
   ui = Ui_MainWindow()
   ui.setupUi(MainWindow)
   MainWindow.show()
   sys.exit(ui.appExec())

   
   
