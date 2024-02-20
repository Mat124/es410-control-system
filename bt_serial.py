import sys
import os

import serial

from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import Qt
from PyQt6 import QtBluetooth

class bluetoothTest(QWidget):

    def __init__(self, parent = None):
        super(bluetoothTest, self).__init__(parent)
        self.connectToRobot()
        self.win = QWidget()
        self.win.show()

    def connectToRobot(self):
        self.sock = QtBluetooth.QBluetoothSocket(QtBluetooth.QBluetoothServiceInfo.Protocol.RfcommProtocol)

        self.sock.connected.connect(self.connectedToBluetooth)
        self.sock.readyRead.connect(self.receivedBluetoothMessage)
        self.sock.disconnected.connect(self.disconnectedFromBluetooth)
        self.sock.errorOccurred.connect(self.socketError)
        port = 1
        self.sock.connectToService(QtBluetooth.QBluetoothAddress("A0:A3:B3:2B:F2:2A"),port)

    def socketError(self,error):
        print(self.sock.errorString())

    def connectedToBluetooth(self):
        print('Connected to bluetooth')
        self.sock.write('A'.encode())

    def disconnectedFromBluetooth(self):
        print('Disconnected from bluetooth')

    def receivedBluetoothMessage(self):
        while self.sock.canReadLine():
            line = self.sock.readLine()
            print(str(line, "utf-8"))
            self.sock.write("OK".encode())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = bluetoothTest()
    app.exec()