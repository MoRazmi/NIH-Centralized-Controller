import sys,os
import serial
import numpy
import can
import time
from time import sleep
from threading import Thread
import threading
from ast import literal_eval
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import Qt
from PyQt5 import  QtCore, QtWidgets, QtSerialPort
from PyQt5.QtSerialPort import QSerialPort
from queue import Queue
import array

#initialzing a queue
SendQ = Queue(maxsize = 10)
ReceiveQ = Queue(maxsize = 10)

#initialize the Torque data in the beginning:
TorqueData = [0] * 2

i = 0

import sqlite3
import serial.tools.list_ports
import logging
#import SerialCommunication

os.system('sudo ip link set can0 type can bitrate 100000')
os.system('sudo ifconfig can0 up')
can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')# socketcan_native

#msg = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], extended_id=False)
#can0.send(msg)


ports = serial.tools.list_ports.comports()


class FirmwareUpgrade(QWidget):



    def __init__(self):

        super().__init__()

        self.setWindowTitle("Fimware Upgrade")
        self.setGeometry(150, 20, 450, 450)
        self.setFixedSize(self.size())
        self.status = False
        
        self.UI()
        self.show()
 #       threading.Thread(target=self.UI()).start()
 #       threading.Thread(target=self.show()).start()
        threading.Thread(target=self.printSensors1).start()
        threading.Thread(target=self.printSensors2).start()
        threading.Thread(target=self.imAlive).start()
#        threading.Thread(target=self.makeConnection).start()
#       t1 = Thread(target=self.demuxTorque)
  #      t1.start()
 #       t1.join()


    def UI(self):
        self.widgets()
        self.layout()
        print([port.name for port in ports])
        self.listEBBports()
        self.findPort()




    def widgets(self):

        self.ComLabel = QLabel("COM Port")
        self.ComLabel.setFont(QFont("Times",18))
        self.ComCombo = QComboBox()
        for info in QtSerialPort.QSerialPortInfo.availablePorts():
            self.ComCombo.addItem(info.portName())


        self.ExoLabel = QLabel("Device Name")
        self.ExoLabel.setFont(QFont("Times", 18))
        self.ExcoCombo = QComboBox()

        self.ComBtn = QPushButton("Command")
        self.ComBtn.clicked.connect((self.makeConnection))

        self.SWStopBtn = QPushButton("Software Stop")
        self.SWStopBtn.clicked.connect(self.SWStop)


        self.MotorEngage = QPushButton("MotorStatus = OFF", self)
        self.MotorEngage.setCheckable(True)
        self.MotorEngage.clicked.connect(self.EngageMotor)
        self.MotorEngage.setStyleSheet("background-color : lightgrey")


        self.maxCurrLabel = QLabel("Max Current")
        self.maxCurrLabel.setFont(QFont("Times", 18))
        
        self.FSRHeel = QLabel("HeelFSR")
        self.FSRHeel.setFont(QFont("Times", 18))
        self.FSRMid = QLabel("MidFSR ")
        self.FSRMid.setFont(QFont("Times", 18))
        self.FSRToe = QLabel("ToeFSR")
        self.FSRToe.setFont(QFont("Times", 18))

        self.AngPosition  = QLabel("AngPos")
        self.AngPosition.setFont(QFont("Times", 18))
        self.RefTorque = QLabel("RefTorq")
        self.RefTorque.setFont(QFont("Times", 18))
        self.Torque = QLabel("Torq")
        self.Torque.setFont(QFont("Times", 18))        
        

        self.maxCurrSlider = QSlider(Qt.Horizontal)
        self.maxCurrSlider.setMinimum(-6000)
        self.maxCurrSlider.setMaximum(6000)
        self.maxCurrSlider.setValue(10)
        self.maxCurrSlider.valueChanged.connect(self.maxCurrUpdate)

        self.maxCurrLCD = QLCDNumber()
        self.PrcStanceLCD = QLCDNumber()

        self.FSRHeelLCD = QLCDNumber()
        self.FSRMidLCD = QLCDNumber()
        self.FSRToeLCD = QLCDNumber()

        self.AngLCD = QLCDNumber()
        self.RefTorqLCD = QLCDNumber()
        self.TorqueLCD = QLCDNumber()
      #  self.TorqueLCD.setDigitCount(12)
        


    def layout(self):
        self.mainLayout = QVBoxLayout()
        self.topLayout = QHBoxLayout()
        self.bottomLayout = QHBoxLayout()
        self.bottom2Layout = QHBoxLayout()
        self.bottom3Layout = QHBoxLayout()
        self.bottom4Layout = QHBoxLayout()
        self.bottom5Layout = QHBoxLayout()
        self.bottom6Layout = QHBoxLayout()
        self.bottom7Layout = QHBoxLayout()


        self.bottomLayout.addWidget(self.maxCurrLabel)
        self.bottomLayout.addWidget(self.maxCurrSlider)
        self.bottom2Layout.addWidget(self.maxCurrLCD)

        self.bottom3Layout.addWidget(self.FSRHeel)
        self.bottom3Layout.addWidget(self.FSRMid)
        self.bottom3Layout.addWidget(self.FSRToe)

        self.bottom4Layout.addWidget(self.FSRHeelLCD)
        self.bottom4Layout.addWidget(self.FSRMidLCD)
        self.bottom4Layout.addWidget(self.FSRToeLCD)

        self.bottom5Layout.addWidget(self.AngPosition)
        self.bottom5Layout.addWidget(self.RefTorque)
        self.bottom5Layout.addWidget(self.Torque)

        self.bottom6Layout.addWidget(self.AngLCD)
        self.bottom6Layout.addWidget(self.RefTorqLCD)
        self.bottom6Layout.addWidget(self.TorqueLCD)

        self.bottom7Layout.addWidget(self.SWStopBtn)


        self.topLayout.addWidget(self.ComBtn,10)
        self.topLayout.addWidget(self.MotorEngage,10)
#        self.topLayout.addWidget(self.ComCombo, 20)
#        self.topLayout.addWidget(self.ScanBtn, 10)

        self.mainLayout.addLayout(self.topLayout,10)
        self.mainLayout.addLayout(self.bottomLayout,60)
        self.mainLayout.addLayout(self.bottom2Layout, 10)
        self.mainLayout.addLayout(self.bottom3Layout, 10)
        self.mainLayout.addLayout(self.bottom4Layout, 10)
        self.mainLayout.addLayout(self.bottom5Layout, 10)
        self.mainLayout.addLayout(self.bottom6Layout, 10)
        self.mainLayout.addLayout(self.bottom7Layout, 10)
      
        self.setLayout(self.mainLayout)


    def EngageMotor(self):

         if self.MotorEngage.isChecked():
         #    self.button.setStyleSheet("background-color : lightgrey")
              self.MotorEngage.setText("On")
              self.status = True
              print(self.status)

         else:
         #    self.button.setStyleSheet("background-color : lightgrey")
              self.MotorEngage.setText("Off")
              self.status = False
              print(self.status)

    def maxCurrUpdate(self,event):
        print(event)
        self.maxCurrLCD.display( self.maxCurrSlider.value())

    def SlewUpUpdate(self,event):
        print(event)
        self.SlewUpLCD.display(self.SlewUpKnob.value() )


    def SlewDownUpdate(self,event):
        print(event)
        self.SlewDownLCD.display(self.SlewDownKnob.value() )

    def PercentStanceUpdate(self,event):
        print(event)
        self.PrcStanceLCD.display((self.PrcStanceKnob.value())/100)


    def listEBBports(self):
        try:
            from serial.tools.list_ports import comports
        except ImportError:
            return None
        if comports:
            com_ports_list = list(comports())
            ebb_ports_list = []
            for port in com_ports_list:
                port_has_ebb = False
                if port[1].startswith("EiBotBoard"):
                    port_has_ebb = True
                elif port[2].startswith("USB VID:PID=04D8:FD92"):
                    port_has_ebb = True
                if port_has_ebb:
                    ebb_ports_list.append(port)
            if ebb_ports_list:
                return ebb_ports_list


    def findPort(self):
        # Find first available EiBotBoard by searching USB ports.
        # Return serial port object.
        try:
            from serial.tools.list_ports import comports
        except ImportError:
            return None
        if comports:
            com_ports_list = list(comports())
            ebb_port = None
            for port in com_ports_list:
                if port[1].startswith("EiBotBoard"):
                    ebb_port = port[0]  # Success; EBB found by name match.
                    break  # stop searching-- we are done.
            if ebb_port is None:
                for port in com_ports_list:
                    if port[2].startswith("USB VID:PID=04D8:FD92"):
                        ebb_port = port[0]  # Success; EBB found by VID/PID match.
                        break  # stop searching-- we are done.
            return ebb_port


    def makeConnection(self):
 #       os.system('sudo ifconfig can0 up')
 #    while True:
 #       can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')
        valueToPut = self.maxCurrSlider.value()
        SendQ.put(valueToPut)
        self.demuxTorque()
        sleep(0.1)
 #       os.system('sudo ifconfig can0 down')
 #       os.system('sudo ifconfig can0 up')
 #       print(hex(TorqueData[1]))
  #      print(hex(TorqueData[0]))



    def demuxTorque(self):
            print(self.status)
            valueToDemux = SendQ.get()
            if valueToDemux > 0 :
                TorqueData[1] = valueToDemux >> 8
                TorqueData[0] = valueToDemux & 255
            else :
               valueToDemux = valueToDemux + 0xFFFF
               TorqueData[1] = valueToDemux >> 8
               TorqueData[0] = valueToDemux & 255

            msg = can.Message(arbitration_id=0x112, data=[self.status, TorqueData[0], TorqueData[1]], extended_id=False)
            can0.send(msg)


    def printSensors1(self):
        global i
        while True:
         msg = can0.recv(10.0)
         if msg.arbitration_id == 280:
            dec1 = int(((msg.data[1] << 8) & 0xFF00)| (msg.data[0] & 0xFF))
            dec2 = int(((msg.data[3] << 8) & 0xFF00)| (msg.data[2] & 0xFF))
            dec3 = int(((msg.data[5] << 8) & 0xFF00)| (msg.data[4] & 0xFF))
            dec4 = (((msg.data[7] << 8) & 0xFF00)| (msg.data[6] & 0xFF))
            if dec4 < 32768:
                dec4 = (int(dec4) * 360.0)/4096
            else :
                dec4 = -((int(0xFFFF - dec4) * 360.0)/4096)
         self.FSRHeelLCD.display(dec1)
         self.FSRMidLCD.display(dec2)
         self.FSRToeLCD.display(dec3)
         self.AngLCD.display(dec4)
    
    def printSensors2(self):
        while True:
         msg = can0.recv(10.0)
         if msg.arbitration_id == 282:
         #   dec1 = int(((msg.data[1] << 8) & 0xFF00)| (msg.data[0] & 0xFF))
            dec1 = ( ((msg.data[3] << 24) & 0xFF000000) | ((msg.data[2] << 16) & 0xFF0000)  | ((msg.data[1] << 8) & 0xFF00)| (msg.data[0] & 0xFF) )
            if dec1 < 524288:
                dec1 = (int(dec1))/32768
            else :
                dec1 = -((0xFFFFFFFF - (int(dec1) ))/32768)
        #    print(dec1)
            self.TorqueLCD.display(round(dec1,2))
            
            dec2 = ( ((msg.data[7] << 24) & 0xFF000000) | ((msg.data[6] << 16) & 0xFF0000)  | ((msg.data[5] << 8) & 0xFF00)| (msg.data[4] & 0xFF) )
            if dec2 < 524288:
                dec2 = (int(dec2))/32768
            else :
                dec2 = -((0xFFFFFFFF - (int(dec2) ))/32768)
       #     print(dec2)
            self.RefTorqLCD.display(round(dec2,2))
     #    self.RefTorqLCD.display(i)
     #    i = i + 1
     
    def imAlive(self):
        count = 0
        while True:
            count += 1
            delay = 0.05  # 20 Hz
            time.sleep(delay)
            if count == 20:
                print("Im Alive")
                count = 0
            msg = can.Message(arbitration_id=0x001, extended_id=False)
            can0.send(msg)

    def SWStop(self):
            
        self.status = False
        self.MotorEngage.setText("Off")
        msg = can.Message(arbitration_id=0x110,  extended_id=False)
        can0.send(msg)
        self.maxCurrSlider.setValue(0)
        self.maxCurrLCD.display( self.maxCurrSlider.value())





    def WriteParameters(self):
        pass


def main():
    App = QApplication(sys.argv)
    window = FirmwareUpgrade()
    sys.exit(App.exec_())
    print(mxCurrent)


if __name__ == '__main__':
   main()

