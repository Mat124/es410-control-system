import tkinter as tk
import threading
from tkinter import ttk

class BaseFrame(tk.Frame):
    '''
    Custom frame class for GUI
    '''
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        # show home page
        homePageButton = ttk.Button(self, text ="Home",
                                    command = lambda : controller.show_frame(Home))
        homePageButton.grid(row = 1, column = 1, padx = 10, pady = 10)

        # show bluetooth setup page
        bluetoothPageButton = ttk.Button(self, text ="Bluetooth Setup",
                                        command = lambda : controller.show_frame(BluetoothSetup))
        bluetoothPageButton.grid(row = 2, column = 1, padx = 10, pady = 10)
  
        # show control scheme choice page
        controlSchemeChoiceButton = ttk.Button(self, text ="Control Scheme Choice",
                                                command = lambda : controller.show_frame(ControlSchemeChoice))
        controlSchemeChoiceButton.grid(row = 3, column = 1, padx = 10, pady = 10)

        # show sensor display page
        sensorDisplayButton = ttk.Button(self, text ="Sensor Display",
                                        command = lambda : controller.show_frame(SensorDisplay))
        sensorDisplayButton.grid(row = 4, column = 1, padx = 10, pady = 10)

class BluetoothSetup(BaseFrame):
    '''
    GUI frame for setting up bluetooth connection to the robot
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Bluetooth Setup", font = ("Verdana", 35))

        self.controllerRef = controller

        self.robotOptions = [
            "N/A",
            "Robot 1",
            "Robot 2",
            "Robot 3"
        ]
        self.currSelected = tk.StringVar()
        self.currSelected.set(self.robotOptions[0])
        self.robotDropdown = ttk.OptionMenu(self, self.currSelected, *self.robotOptions)
        self.robotDropdown.grid(row=4, column=5)

        self.connectionThread = threading.Thread(target=self.tryConnect)
        self.bluetoothConnectButton = ttk.Button(self, text ="Connect to Robot",
                                            command = self.connectionThread.run)
        self.bluetoothConnectButton.grid(row = 5, column = 5)

        self.btStatusLabelText = tk.StringVar()
        self.btStatusLabel = ttk.Label(self, textvariable=self.btStatusLabelText)
        self.btStatusLabel.grid(row=6, column=5)
        
        # putting the grid in its place by using
        # grid
        label.grid(row = 0, column = 4)

    def tryConnect(self):
        print(self.currSelected.get())
        if self.currSelected.get() != "N/A":
            self.btStatusLabelText.set("Establishing connection")
            self.controllerRef.bt_comm.connect(self.currSelected.get()[-1],)
            self.btStatusLabelText.set("Connected")
        else:
            self.btStatusLabelText.set("Please select a robot from the dropdown box!")

class ControlSchemeChoice(BaseFrame):
    '''
    GUI frame for choosing control scheme
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Control Scheme Choice", font = ("Verdana", 35))
        
        # putting the grid in its place by using
        # grid
        label.grid(row = 0, column = 4) 

class SensorDisplay(BaseFrame):
    '''
    GUI frame for displaying sensor data
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Sensor Display", font = ("Verdana", 35))
        label.grid(row = 0, column = 4)

        self.displaySensorData = {
            "Battery Temperature": -1.0,
            "Battery Voltage": -1.0,
            "Battery Percentage": -1.0,
            "Current Velocity": -1.0,
            "Current Acceleration": -1.0
        }
        
        i = 5
        for key, value in self.displaySensorData.items():
            label = ttk.Label(self, text = f"{key}:")
            label.grid(row = i, column = 4)
            label = ttk.Label(self, text = f"{value}")
            label.grid(row = i, column = 5)
            i += 1

class Home(BaseFrame):
    '''
    GUI frame for main page
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Home", font = ("Verdana", 35))
        
        # putting the grid in its place by using
        # grid
        label.grid(row = 0, column = 4) 
