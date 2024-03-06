import time
import threading

from enum import Enum
from bt_client import bluetoothCommunicator
        
def sensorLogger(BT):
    while True:
        BT.sensorDataExists.wait()
        BT.sensorDataLock.acquire()
        while len(BT.sensorData) > 0:
            print("Received sensor data: ", BT.sensorData.pop(0))
        BT.sensorDataExists.clear() # clear inside of lock to prevent sensor reader thread from setting and then this thread clearing
        BT.sensorDataLock.release()
        time.sleep(0.1)

class sensor(Enum):
    VOLTAGE = 'V'.encode()
    TEMPERATURE = 'T'.encode()
    X_ACCEL = 'X'.encode()
    Y_ACCEL = 'Y'.encode()
    Z_ACCEL = 'Z'.encode()
    X_GYRO = 'x'.encode()
    Y_GYRO = 'y'.encode()
    Z_GYRO = 'z'.encode()

if __name__ == "__main__":
    print(sensor.VOLTAGE.value[0])

    BT = bluetoothCommunicator()

    BT.connect()

    # start sensor thread

    motorLed = input("Enter 1 for motor control, 2 for LED control: ")

    prepend = ""

    if motorLed == "1":
        prepend = "R"
    else:
        prepend = "Z"

    while True:
        msg = input("Enter speed [0-1] or 'exit': ")
        if msg == "exit":
            break
        BT.sendSerialMsg(prepend + msg + "\n")
    