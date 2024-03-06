import inputs
import threading

class GamepadModel:
    # holds the current state of the gamepad

    # joysticks, buttons and triggers
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
    
    def normaliseInput(self, event): # normalise input to range [-1, 1]
        match event.code:
            case "ABS_X" | "ABS_Y" | "ABS_RX" | "ABS_RY":
                return event.state / 32768.0
            case "ABS_RZ" | "ABS_Z":
                return event.state / 1023.0
            case _:
                return event.state

    def updateModel(self, events):
        for event in events:
            self.values[event.code] = self.normaliseInput(event)

    def __init__(self):
        pass


class GamepadParser:

    # control schemes, each with a mapping of gamepad inputs to motor outputs
    control_schemes = [ {
        "turn": "ABS_X",
        "forward": "ABS_RZ",
        "reverse": "ABS_Z",
        "weapon": "BTN_SOUTH",
        "toggle_auto_weapon": "BTN_WEST",
        "toggle_auto_movement": "BTN_NORTH",
        "led": "ABS_RY"
        }
    ]

    motorOutputs = {
        "left": 0,
        "right": 0,
        "weapon": 0,
        "led": 0
    } # output range [-1, 1]

    autonomousWeapon = False
    autonomousWeaponHeld = False
    autonomousMovement = False
    autonomousMovementHeld = False

    def updateControlScheme(self, control_scheme):
        self.control_scheme = control_scheme

    def updateOutputs(self): # map turning and acceleration to motor outputs
        turn = self.control_schemes[self.control_scheme]["turn"]
        forward = self.control_schemes[self.control_scheme]["forward"]
        reverse = self.control_schemes[self.control_scheme]["reverse"]
        weapon = self.control_schemes[self.control_scheme]["weapon"]
        auto_weapon = self.control_schemes[self.control_scheme]["toggle_auto_weapon"]
        auto_movement = self.control_schemes[self.control_scheme]["toggle_auto_movement"]
        led = self.control_schemes[self.control_scheme]["led"]

        # toggle autonomous weapon
        if self.gamepadModel.values[auto_weapon] and not self.autonomousWeaponHeld:
            self.autonomousWeapon = not self.autonomousWeapon
            self.autonomousWeaponHeld = True
        
        # toggle autonomous movement
        if self.gamepadModel.values[auto_movement] and not self.autonomousMovementHeld:
            self.autonomousMovement = not self.autonomousMovement
            self.autonomousMovementHeld = True

        if not self.gamepadModel.values[auto_weapon]:
            self.autonomousWeaponHeld = False
        if not self.gamepadModel.values[auto_movement]:
            self.autonomousMovementHeld = False

        # set outputs based on gamepad inputs
        self.motorOutputs["left"] = self.gamepadModel.values[turn] + self.gamepadModel.values[forward] - self.gamepadModel.values[reverse]
        self.motorOutputs["right"] = -self.gamepadModel.values[turn] + self.gamepadModel.values[forward] - self.gamepadModel.values[reverse]
        self.motorOutputs["weapon"] = self.gamepadModel.values[weapon]
        self.motorOutputs["led"] = abs(self.gamepadModel.values[led])

        # cap at [-1, 1] in the case of controller error
        for output in self.motorOutputs:
            self.motorOutputs[output] = max(min(self.motorOutputs[output], 1), -1)

    def read_gamepad(self): # read gamepad inputs and update model
        while True:
            events = self.gamepad.read() # block until new input
            self.gamepadModel.updateModel(events)
            self.updateOutputs()

    def __init__(self, control_scheme = 0):
        try:
            # only attempt connection if a gamepad is connected
            if len(inputs.devices.gamepads) > 0:
                self.gamepad = inputs.devices.gamepads[0]
        except Exception as e:
            raise e
        
        # create gamepad model and set control scheme
        self.gamepadModel = GamepadModel()
        self.updateControlScheme(control_scheme)

        # start gamepad reader thread
        t = threading.Thread(target=self.read_gamepad)
        t.daemon = True # thread stops when main thread stops
        t.start()