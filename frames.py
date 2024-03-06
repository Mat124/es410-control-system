import tkinter as tk
import threading
import time
from tkinter import ttk

class BaseFrame(tk.Frame):
    '''
    Custom frame class for GUI
    This is a virtual class, and should not be instantiated
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
        label.grid(row = 0, column = 4)

        self.controllerRef = controller # save reference to controller for use in other methods

        self.robotOptions = [
            "N/A",
            "Robot 1",
            "Robot 2",
            "Robot 3"
        ]
        self.currSelected = tk.StringVar()
        self.currSelected.set(self.robotOptions[0])
        # create dropdown box for selecting robot
        self.robotDropdown = ttk.OptionMenu(self, self.currSelected, *self.robotOptions)
        self.robotDropdown.grid(row=4, column=5)

        # create a thread to connect to the robot, preventing the GUI from freezing
        self.connectionThread = threading.Thread(target=self.tryConnect)
        self.bluetoothConnectButton = ttk.Button(self, text ="Connect to Robot",
                                            command = self.connectionThread.run)
        self.bluetoothConnectButton.grid(row = 5, column = 5)

        # label to see the status of the bluetooth connection
        self.btStatusLabelText = tk.StringVar()
        self.btStatusLabel = ttk.Label(self, textvariable=self.btStatusLabelText)
        self.btStatusLabel.grid(row=6, column=5)

    def disconnect(self):
        pass

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
        label.grid(row = 0, column = 4) 

class SensorDisplay(BaseFrame):
    '''
    GUI frame for displaying sensor data
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Sensor Display", font = ("Verdana", 35))
        label.grid(row = 0, column = 4)

        self.displaySensorData = controller.sensorData

        self.updateSensorDataEvent = threading.Event()
        self.updateSensorDataEvent.clear()

        i = 5
        self.labels = {}
        for key, value in self.displaySensorData.items():
            label = ttk.Label(self, text = f"{key}:")
            label.grid(row = i, column = 4)
            self.labels[key] = tk.StringVar()
            label = ttk.Label(self, textvariable = self.labels[key])
            label.grid(row = i, column = 5)
            i += 1

    def updateSensorData(self):
        # thread runs this code and updates sensor data every 100ms until the event is cleared
        while self.updateSensorDataEvent.is_set():
            for key, value in self.displaySensorData.items():
                self.labels[key].set(f"{value}")
            time.sleep(0.1)
    
    def tkraise(self) -> None:
        # start the sensor data update thread when the frame is raised (shown)
        self.updateSensorDataEvent.set()
        self.updateSensorDataThread = threading.Thread(target=self.updateSensorData)
        self.updateSensorDataThread.start()
        return super().tkraise()

class Home(BaseFrame):
    '''
    GUI frame for main page
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Home", font = ("Verdana", 35))
        label.grid(row = 0, column = 4) 
