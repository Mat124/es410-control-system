import asyncio

from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import Qt
from PyQt6 import QtBluetooth

class BTHandler():

    def __init__(self):
        self.app = QApplication([])
        self.window = QWidget()
        self.window.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint)
        self.window.show()
        self.discoveryAgent = QtBluetooth.QBluetoothDeviceDiscoveryAgent()
        self.discoveryAgent.deviceDiscovered.connect(self.onDeviceDiscovered)
        self.discoveryAgent.finished.connect(self.onScanFinished)
        self.discoveryAgent.start()

    def onDeviceDiscovered(self, device):
        if device.majorDeviceClass() == QtBluetooth.QBluetoothDeviceInfo.MajorDeviceClass.ComputerDevice or device.majorDeviceClass() == QtBluetooth.QBluetoothDeviceInfo.MajorDeviceClass.PhoneDevice or device.majorDeviceClass() == QtBluetooth.QBluetoothDeviceInfo.MajorDeviceClass.WearableDevice:
            print(device.name(), device.address().toString(), device.rssi(), device.isValid(), device.coreConfigurations(), device.majorDeviceClass())
        
    def onScanFinished(self):
        print("Scan finished")
        self.app.exit()

    def run(self):
        self.app.exec()

if __name__ == "__main__":
    bt = BTHandler()
    bt.run()