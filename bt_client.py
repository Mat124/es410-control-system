import sys
import threading
import time

import socket

from PyQt6.QtCore import QCoreApplication
from PyQt6 import QtBluetooth

class bluetoothCommunicator():

    def __init__(self, parent = None):
        self.sock = None
        print("Starting bluetooth test")
        app = QCoreApplication(sys.argv)

    def connect(self):
        self.deviceInfo = None
        self.sensorDataLock = threading.Lock()
        self.sensorData = []
        self.sensorDataExists = threading.Event()
        self.scanForDevices()

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
            while self.connectToRobot():
                print("Error connecting to robot, retrying")
                time.sleep(1)

    def connectToRobot(self):
        print("Setting up socket")
        self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        print("Connecting to device")
        try:
            self.sock.connect((self.deviceInfo.address().toString(), 1))
        except Exception as e:
            print("Error connecting to device: ", e)
            self.sock = None
            return 1
        sockReadThread = threading.Thread(target=self.readBluetoothMessage)
        sockReadThread.start()
        return 0

    def readBluetoothMessage(self):
        while self.sock is not None:
            try:
                data = self.sock.recv(4096)
                print("Received data: ", data)
                self.sensorDataLock.acquire()
                self.sensorData.append(data)
                self.sensorDataExists.set()
                self.sensorDataLock.release()
            except Exception as e:
                print("Error reading from socket: ", e)
                self.sock = None
                return
            
    def sendMotorControl(self, motorOutputs): # dict: left, right, weapon
        if self.sock is not None:
            try:
                ledData = "Z" + str(motorOutputs["led"])
                leftData = "L" + str(motorOutputs["left"])
                rightData = "R" + str(motorOutputs["right"])
                weaponData = "W" + str(motorOutputs["weapon"])
                sendMsg = ledData + " " + leftData + " " + rightData + " " + weaponData + "\n"
                self.sock.send(sendMsg.encode())
            except Exception as e:
                print("Error sending motor control: ", e)
                self.sock = None
                return
            
    def sendSerialMsg(self, msg):
        if self.sock is not None:
            try:
                self.sock.send(msg.encode())
            except Exception as e:
                print("Error sending serial message: ", e)
                self.sock = None
                return
