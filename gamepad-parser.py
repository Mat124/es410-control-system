import inputs
import threading

control_schemes = { {
        "ABS_X": "turn", # -1 to 1 (left to right)
        "ABS_RZ": "forward", # 0 to 1
        "ABS_Z": "reverse", # 0 to 1
        "BTN_SOUTH": "weapon" # 0 to 1 (probably just 0 or 1)
    }
}

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
    "BTN_WEST": 0
    }

    def __init__(self):
        pass
    
    def update_model(self, events):
        for event in events:
            self.values[event.code] = normalise_input(event.state)

    def normalise_input(self, event): # normalise input to range [-1, 1]
        match event.code:
            case "ABS_X" | "ABS_Y" | "ABS_RX" | "ABS_RY":
                return event.state / 32768.0
            case "ABS_RZ" | "ABS_Z":
                return event.state / 1024.0
            case _:
                return event.state


class GamepadParser:

    motor_outputs = {
        "left": 0,
        "right": 0,
        "weapon": 0
    } # output range [-1, 1]

    def __init__(self, control_scheme = 0):
        self.gamepad = inputs.devices.gamepads[0]
        self.gamepad_model = GamepadModel()
        update_control_scheme(control_scheme)

        threading.Thread(target=self.read_gamepad).start()

    def update_control_scheme(self, control_scheme):
        self.control_scheme = control_scheme

    def update_motor_outputs(self): # map turning and acceleration to motor outputs
        turn = control_schemes[self.control_scheme]["turn"]
        forward = control_schemes[self.control_scheme]["forward"]
        reverse = control_schemes[self.control_scheme]["reverse"]
        weapon = control_schemes[self.control_scheme]["weapon"]

        self.motor_outputs["left"] = self.gamepad_model.values[turn] + self.gamepad_model.values[forward]

    def read_gamepad(self): # read gamepad inputs and update model
        while True:
            events = self.gamepad.read()
            self.gamepad_model.update_model(events)
            self.update_motor_outputs()