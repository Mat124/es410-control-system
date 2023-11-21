import gamepad_parser
import tkinter as tk
from tkinter import ttk

class GUI(tk.Tk):
    '''
    GUI class for the Fighting Robot Controller

    Need to include:
    - Control scheme choice
        - maybe a controller visualisation to show which buttons do what
    - Sensor display
        - Battery levels, current temperatures, motor outputs, etc.
        - Warnings for low battery, high temps, loss of sensor signals, etc.
    - Bluetooth connection setup to the robot
    
    '''

    

    def send_motor_outputs(self):
        '''
        Send motor outputs to the robot
        '''
        motor_out = {
            "left": 0.0,
            "right": 0.0,
            "weapon": 0.0
        }

        motor_out["weapon"] = self.gamepad.motor_outputs["weapon"]
        # if self.gamepad.autonomousWeapon: motor_out["weapon"] += self.autonomous_weapon_output # TODO: implement autonomous weapon control

        if self.gamepad.autonomousMovement:
            pass
            # TODO: implement autonomous movement control
        else:
            motor_out["left"] = self.gamepad.motor_outputs["left"]
            motor_out["right"] = self.gamepad.motor_outputs["right"]

        # self.robot_communicator.send_motor_outputs(motor_out) # TODO: implement robot communication
    
    def __init__(self):
        '''
        Initialise GUI
        Create gamepad parser, bluetooth connection, sensor data logger, robot QR code reader, etc.
        '''
        self.gamepad = gamepad_parser.GamepadParser() # initialise gamepad parser
        
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

        self.mainloop()
    
    def show_frame(self, cont):
        '''
        Show frame
        '''
        frame = self.frames[cont]
        frame.tkraise()

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

if __name__ == "__main__":
    gui = GUI()
