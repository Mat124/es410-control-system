import threading
import tkinter as tk
import time

from frames import *
from bt_client import bluetoothCommunicator
from gamepad_parser import GamepadParser

class App(tk.Tk):
    '''
    App class for the Fighting Robot Controller

    Initialises the gamepad parser, bluetooth connection and GUI.
    Starts a thread to send motor outputs to the robot.
    '''
    
    def __init__(self):
        '''
        Initialise App
        Create gamepad parser, bluetooth connection, sensor data logger, robot QR code reader, etc.
        '''
        self.gamepad = GamepadParser() # initialise gamepad parser

        self.bt_comm = bluetoothCommunicator() # initialise bluetooth communicator

        self.motorOut = {
            "left": 0.0,
            "right": 0.0,
            "weapon": 0.0,
            "led": 0.0
        }

        self.sensorData = self.bt_comm.sensorData

        self.sendMotorOutputs = True

        self.lastSent = time.time()
        
        # setup GUI
        tk.Tk.__init__(self)
        container = tk.Frame(self)
        container.pack(side = "top", fill = "both", expand = True) 
        container.grid_rowconfigure(0, weight = 1)
        container.grid_columnconfigure(0, weight = 1)

        self.frames = {}

        for F in (Home, BluetoothSetup, ControlSchemeChoice, SensorDisplay):
            frame = F(container, self)
            self.frames[F] = frame 
            frame.grid(row = 0, column = 0, sticky ="nsew")
  
        self.show_frame(Home)

        self.title("Fighting Robot Controller")
        self.geometry("800x800")

        self.sendMotorOutputsThread = threading.Thread(target=self.send_motor_outputs)
        self.sendMotorOutputsThread.start()

        self.mainloop()
    
    def show_frame(self, cont):
        '''
        Show frame
        '''
        frame = self.frames[cont]
        if cont != SensorDisplay:
            self.frames[SensorDisplay].updateSensorDataEvent.clear()
        frame.tkraise()


    def send_motor_outputs(self):
        '''
        Send motor outputs to the robot
        '''
        while self.sendMotorOutputs:
            oldMotorOut = {}
            for key, value in self.motorOut.items():
                oldMotorOut[key] = value

            self.motorOut["weapon"] = self.gamepad.motorOutputs["weapon"]
            # if self.gamepad.autonomousWeapon: motor_out["weapon"] += self.autonomous_weapon_output # TODO: implement autonomous weapon control

            if self.gamepad.autonomousMovement:
                pass
                # TODO: implement autonomous movement control
            else:
                self.motorOut["left"] = self.gamepad.motorOutputs["left"]
                self.motorOut["right"] = self.gamepad.motorOutputs["right"]
                self.motorOut["led"] = self.gamepad.motorOutputs["led"] # delete this after test
                
            for key in self.motorOut:
                if self.motorOut[key] != oldMotorOut[key] and time.time() - self.lastSent > 0.01:
                    self.bt_comm.sendMotorControl(self.motorOut)
                    self.lastSent = time.time()
                    break

if __name__ == "__main__":
    app = App()
