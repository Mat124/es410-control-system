import sys
import threading
import time

import socket

from PyQt6.QtCore import QCoreApplication
from PyQt6 import QtBluetooth
from enum import Enum

# enum for connection status, more readable than using numbers
class status(Enum):
    UNCONNECTED = 0
    CONNECTED = 1
    CONNECTING = 2
    CONNECTIONERROR = -1

# enum for sensor data, more readable than characters
class sensor(Enum):
    VOLTAGE = 'V'.encode()[0]
    TEMPERATURE = 'T'.encode()[0]
    DISTANCE = 'D'.encode()[0]
    X_ACCEL = 'X'.encode()[0]
    Y_ACCEL = 'Y'.encode()[0]
    Z_ACCEL = 'Z'.encode()[0]
    X_GYRO = 'x'.encode()[0]
    Y_GYRO = 'y'.encode()[0]
    Z_GYRO = 'z'.encode()[0]

class bluetoothCommunicator():
    '''
    Class for handling bluetooth communication with the robot
    Uses the PyQT6 library for identifying the robot, and the socket library for communication
    '''

    def __init__(self, parent = None):
        self.sock = None
        self.connectionStatus = status.UNCONNECTED
        self.sensorData = {
            "Battery Temperature": -1.0,
            "Battery Voltage": -1.0,
            "Battery Percentage": -1.0,
            "Distance": -1.0,
            "X Acceleration": -1.0,
            "Y Acceleration": -1.0,
            "Z Acceleration": -1.0,
            "X Gyroscope": -1.0,
            "Y Gyroscope": -1.0,
            "Z Gyroscope": -1.0
        }
        app = QCoreApplication(sys.argv)

    def connect(self, char):
        self.connectionStatus = status.CONNECTING
        self.deviceInfo = None
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
        # on device discovery, check if the device is the robot and attempt to connect
        print("Device discovered: ", device.name())
        if device.name().startswith("Robot"):
            print("Found robot device")
            self.deviceInfo = device
            self.discoveryAgent.stop()
            while self.connectToRobot():
                print("Error connecting to robot, retrying")
                self.connectionStatus = status.CONNECTIONERROR
                time.sleep(1)

    def connectToRobot(self):
        print("Setting up socket")
        self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        print("Connecting to device")
        try:
            self.sock.connect((self.deviceInfo.address().toString(), 1))
            self.connectionStatus = status.CONNECTED
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
                sections = data.split(b' ')
                for section in sections:
                    match section[0]:
                        case sensor.VOLTAGE.value:
                            self.sensorData["Battery Voltage"] = float(section[1:])
                            # TODO: map voltage to percentage
                        case sensor.TEMPERATURE.value:
                            self.sensorData["Battery Temperature"] = float(section[1:])
                        case sensor.X_ACCEL.value:
                            self.sensorData["X Acceleration"] = float(section[1:])
                        case sensor.Y_ACCEL.value:
                            self.sensorData["Y Acceleration"] = float(section[1:])
                        case sensor.Z_ACCEL.value:
                            self.sensorData["Z Acceleration"] = float(section[1:])
                        case sensor.X_GYRO.value:
                            self.sensorData["X Gyroscope"] = float(section[1:])
                        case sensor.Y_GYRO.value:
                            self.sensorData["Y Gyroscope"] = float(section[1:])
                        case sensor.Z_GYRO.value:
                            self.sensorData["Z Gyroscope"] = float(section[1:])
                        case sensor.DISTANCE.value:
                            self.sensorData["Distance"] = float(section[1:])
                        case _:
                            print("Unknown sensor data starter: ", section[0])
            except Exception as e:
                print("Error reading from socket: ", e)
                self.sock = None
                return
            
    def sendMotorControl(self, motorOutputs): # motorOutputs: dictionary {led, left, right, weapon}
        if self.sock is not None: # only send if connected
            try:
                ledData = "Z" + str(motorOutputs["led"])
                leftData = "L" + str(motorOutputs["left"])
                rightData = "R" + str(motorOutputs["right"])
                weaponData = "W" + str(motorOutputs["weapon"])
                # concatenate the data into a single string and send
                sendMsg = ledData + " " + leftData + " " + rightData + " " + weaponData + "\n"
                self.sock.send(sendMsg.encode())
            except Exception as e:
                print("Error sending motor control: ", e)
                self.sock = None
                return
            
    def sendSerialMsg(self, msg): 
        # send a custom serial message to the robot for testing purposes
        if self.sock is not None:
            try:
                self.sock.send(msg.encode())
            except Exception as e:
                print("Error sending serial message: ", e)
                self.sock = None
                return
