import sys
import threading
import time

import socket

from PyQt6.QtCore import QCoreApplication
from PyQt6 import QtBluetooth

sock = None
serviceDiscovery = None

class bluetoothCommunicator():

    def __init__(self, parent = None):
        print("Starting bluetooth test")
        app = QCoreApplication(sys.argv)
        self.deviceInfo = None
        self.scanForDevices()
        self.sensorDataLock = threading.Lock()
        self.sensorData = []
        app.exec()

    def scanForDevices(self):
        self.discoveryAgent = QtBluetooth.QBluetoothDeviceDiscoveryAgent()
        self.discoveryAgent.deviceDiscovered.connect(self.deviceDiscovered)
        self.discoveryAgent.errorOccurred.connect(self.discoveryError)
        self.discoveryAgent.finished.connect(self.discoveryStopped)
        self.discoveryAgent.start(QtBluetooth.QBluetoothDeviceDiscoveryAgent().DiscoveryMethod.ClassicMethod)

    def discoveryStopped(self):
        print("Discovery stopped")

    def discoveryError(self, error):
        print("Error: ", error)

    def deviceDiscovered(self, device):
        print("Device discovered: ", device.name())
        if device.name().startswith("Robot"):
            print("Found robot device")
            self.deviceInfo = device
            self.discoveryAgent.stop()
            self.connectToRobot()

    def connectToRobot(self):
        print("Setting up socket")
        self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        print("Connecting to service")
        self.sock.connect((self.deviceInfo.address().toString(), 1))
        sockReadThread = threading.Thread(target=self.readBluetoothMessage)
        sockReadThread.start()

    def readBluetoothMessage(self):
        while self.sock.:
            data = self.sock.recv(4096)
            self.sensorDataLock.acquire()
            self.sensorData.append(data)
            self.sensorDataLock.release()

if __name__ == "__main__":
    BT = bluetoothCommunicator()