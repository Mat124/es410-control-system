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
                self.sensorDataLock.acquire()
                self.sensorData.append(data)
                self.sensorDataLock.release()
            except Exception as e:
                print("Error reading from socket: ", e)
                self.sock = None
                return
            
    def sendMotorControl(self, motorOutputs): # dict: left, right, weapon
        if self.sock is not None:
            try:
                leftData = "L" + str(motorOutputs["left"]) + "\n"
                rightData = "R" + str(motorOutputs["right"]) + "\n"
                weaponData = "W" + str(motorOutputs["weapon"]) + "\n"
                self.sock.send(leftData.encode())
                self.sock.send(rightData.encode())
                self.sock.send(weaponData.encode())
            except Exception as e:
                print("Error sending motor control: ", e)
                self.sock = None
                return
        

if __name__ == "__main__":
    BT = bluetoothCommunicator()

    # wait for keyboard input
    input("Press Enter to continue...")

    # send motor control
    motorOutputs = {"left": 0.5, "right": 0.5, "weapon": 0.5}
    BT.sendMotorControl(motorOutputs)

    # wait for keyboard input
    input("Press Enter to continue...")

    # wait for sensor data
    while True:
        BT.sensorDataLock.acquire()
        while len(BT.sensorData) > 0:
            print("Sensor data: ", BT.sensorData.pop(0))
        BT.sensorDataLock.release()
        time.sleep(1)