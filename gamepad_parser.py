import inputs
import threading

class GamepadModel:

    # need to think about joysticks - don't know if linear or not

    # just implementing joysticks, buttons and triggers for now
    values = {
    "ABS_X": 0,
    "ABS_Y": 0,
    "ABS_RX": 0,
    "ABS_RY": 0,
    "ABS_Z": 0,
    "ABS_RZ": 0,
    "BTN_SOUTH": 0,
    "BTN_EAST": 0,
    "BTN_NORTH": 0,
    "BTN_WEST": 0,
    "BTN_TR": 0,
    "BTN_TL": 0
    }
    
    def normalise_input(self, event): # normalise input to range [-1, 1]
        match event.code:
            case "ABS_X" | "ABS_Y" | "ABS_RX" | "ABS_RY":
                return event.state / 32768.0
            case "ABS_RZ" | "ABS_Z":
                return event.state / 1024.0
            case _:
                return event.state

    def update_model(self, events):
        for event in events:
            self.values[event.code] = self.normalise_input(event)

    def __init__(self):
        pass


class GamepadParser:

    control_schemes = [ {
        "turn": "ABS_X", # -1 to 1 (left to right)
        "forward": "ABS_RZ", # 0 to 1
        "reverse": "ABS_Z", # 0 to 1
        "weapon": "BTN_SOUTH" # 0 to 1 (probably just 0 or 1)
        }
    ]

    motor_outputs = {
        "left": 0,
        "right": 0,
        "weapon": 0
    } # output range [-1, 1]

    def update_control_scheme(self, control_scheme):
        self.control_scheme = control_scheme

    def update_motor_outputs(self): # map turning and acceleration to motor outputs
        turn = self.control_schemes[self.control_scheme]["turn"]
        forward = self.control_schemes[self.control_scheme]["forward"]
        reverse = self.control_schemes[self.control_scheme]["reverse"]
        weapon = self.control_schemes[self.control_scheme]["weapon"]

        self.motor_outputs["left"] = self.gamepad_model.values[turn] + self.gamepad_model.values[forward] - self.gamepad_model.values[reverse]
        self.motor_outputs["right"] = -self.gamepad_model.values[turn] + self.gamepad_model.values[forward] - self.gamepad_model.values[reverse]
        self.motor_outputs["weapon"] = self.gamepad_model.values[weapon]

        for output in self.motor_outputs:
            self.motor_outputs[output] = max(min(self.motor_outputs[output], 1), -1)

    def read_gamepad(self): # read gamepad inputs and update model
        while True:
            events = self.gamepad.read()
            self.gamepad_model.update_model(events)
            self.update_motor_outputs()

    def __init__(self, control_scheme = 0):
        self.gamepad = inputs.devices.gamepads[0]
        self.gamepad_model = GamepadModel()
        self.update_control_scheme(control_scheme)

        threading.Thread(target=self.read_gamepad).start()