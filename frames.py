import tkinter as tk
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

        bluetoothConnectButton = ttk.Button(self, text ="Connect to Robot",
                                            command = controller.bt_comm.connect)
        bluetoothConnectButton.grid(row = 5, column = 4, padx = 10, pady = 10)
        
        # putting the grid in its place by using
        # grid
        label.grid(row = 0, column = 4, padx = 10, pady = 10)

class ControlSchemeChoice(BaseFrame):
    '''
    GUI frame for choosing control scheme
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Control Scheme Choice", font = ("Verdana", 35))
        
        # putting the grid in its place by using
        # grid
        label.grid(row = 0, column = 4, padx = 10, pady = 10) 

class SensorDisplay(BaseFrame):
    '''
    GUI frame for displaying sensor data
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Sensor Display", font = ("Verdana", 35))
        
        # putting the grid in its place by using
        # grid
        label.grid(row = 0, column = 4, padx = 10, pady = 10) 

class Home(BaseFrame):
    '''
    GUI frame for main page
    '''
    def __init__(self, parent, controller):
        BaseFrame.__init__(self, parent, controller)

        label = ttk.Label(self, text ="Home", font = ("Verdana", 35))
        
        # putting the grid in its place by using
        # grid
        label.grid(row = 0, column = 4, padx = 10, pady = 10) 
