import gamepad_parser
import tkinter as tk

class GUI:
    '''
    GUI class for the Fighting Robot Controller

    Need to include:
    - Control scheme choice
        - maybe a controller visualisation to show which buttons do what
    - Motor output display
    - Sensor display
        - Battery levels, current temperatures
        - Warnings for low battery, high temps, loss of sensor signals, etc.
    - Bluetooth connection setup to the robot
    
    '''

    def send_motor_outputs(self):
        '''
        Send motor outputs to the robot
        '''
        if self.gamepad.autonomousWeapon:
            pass
        if self.gamepad.autonomousMovement:
            pass
    
    def __init__(self):
        '''
        Initialise GUI
        Create gamepad parser, bluetooth connection, sensor data logger, robot QR code reader, etc.
        '''
        self.gamepad = gamepad_parser.GamepadParser() # initialise gamepad parser

        self.window = tk.Tk()
        self.window.title("Fighting Robot Controller")
        self.window.geometry("800x800")


        self.window.mainloop()

if __name__ == "__main__":
    gui = GUI()