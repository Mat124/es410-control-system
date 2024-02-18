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
                return event.state / 1023.0
            case _:
                return event.state

    def update_model(self, events):
        for event in events:
            self.values[event.code] = self.normalise_input(event)

    def __init__(self):
        pass


class GamepadParser:

    control_schemes = [ {
        "turn": "ABS_X",
        "forward": "ABS_RZ",
        "reverse": "ABS_Z",
        "weapon": "BTN_SOUTH",
        "toggle_auto_weapon": "BTN_WEST",
        "toggle_auto_movement": "BTN_NORTH"
        }
    ]

    motor_outputs = {
        "left": 0,
        "right": 0,
        "weapon": 0
    } # output range [-1, 1]

    autonomousWeapon = False
    autonomousWeaponHeld = False
    autonomousMovement = False
    autonomousMovementHeld = False

    def update_control_scheme(self, control_scheme):
        self.control_scheme = control_scheme

    def update_outputs(self): # map turning and acceleration to motor outputs
        turn = self.control_schemes[self.control_scheme]["turn"]
        forward = self.control_schemes[self.control_scheme]["forward"]
        reverse = self.control_schemes[self.control_scheme]["reverse"]
        weapon = self.control_schemes[self.control_scheme]["weapon"]
        auto_weapon = self.control_schemes[self.control_scheme]["toggle_auto_weapon"]
        auto_movement = self.control_schemes[self.control_scheme]["toggle_auto_movement"]

        if self.gamepad_model.values[auto_weapon] and not self.autonomousWeaponHeld:
            self.autonomousWeapon = not self.autonomousWeapon
            self.autonomousWeaponHeld = True
        
        if self.gamepad_model.values[auto_movement] and not self.autonomousMovementHeld:
            self.autonomousMovement = not self.autonomousMovement
            self.autonomousMovementHeld = True

        if not self.gamepad_model.values[auto_weapon]:
            self.autonomousWeaponHeld = False
        if not self.gamepad_model.values[auto_movement]:
            self.autonomousMovementHeld = False

        self.motor_outputs["left"] = self.gamepad_model.values[turn] + self.gamepad_model.values[forward] - self.gamepad_model.values[reverse]
        self.motor_outputs["right"] = -self.gamepad_model.values[turn] + self.gamepad_model.values[forward] - self.gamepad_model.values[reverse]
        self.motor_outputs["weapon"] = self.gamepad_model.values[weapon]

        for output in self.motor_outputs:
            self.motor_outputs[output] = max(min(self.motor_outputs[output], 1), -1)

    def read_gamepad(self): # read gamepad inputs and update model
        while True:
            events = self.gamepad.read()
            self.gamepad_model.update_model(events)
            self.update_outputs()

    def __init__(self, control_scheme = 0):
        try:
            if len(inputs.devices.gamepads) > 0:
                self.gamepad = inputs.devices.gamepads[0]
        except Exception as e:
            # raise("Issue connecting to gamepad.") # TODO: implement error handling - want to send this to GUI
            raise e

        self.gamepad_model = GamepadModel()
        self.update_control_scheme(control_scheme)

        t = threading.Thread(target=self.read_gamepad)
        t.daemon = True # thread stops when main thread stops
        t.start()