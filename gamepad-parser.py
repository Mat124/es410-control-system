import inputs
import threading

control_schemes = { {
        "ABS_X": "turn",
        "ABS_RZ": "forward",
        "ABS_Z": "reverse",
        "BTN_SOUTH": "weapon"
    }
}

class GamepadModel:
    
    def update_model(self, event):
        # update model with event
        # call update_motor_outputs
        pass

    def update_motor_outputs(self):
        # update motor outputs based on model
        pass

class GamepadParser:

    motor_outputs = [0.0, 0.0, 0.0]

    def __init__(self, control_scheme = 0):
        self.gamepad = inputs.devices.gamepads[0]
        update_control_scheme(control_scheme)

        threading.Thread(target=self.read_gamepad).start()

    def update_control_scheme(self, control_scheme):
        self.control_scheme = control_scheme

    def read_gamepad(self):
        while True:
            events = self.gamepad.read()
            for event in events:
                parsed_event = control_schemes[self.control_scheme][event.code]
                value = normalise_input(event)

                match parsed_event:
                    case "turn":
                        motor_outputs[0] = value
                        motor_outputs[1] = -value
    
    def normalise_input(self, event):
        match event.code:
            case "ABS_X" | "ABS_Y" | "ABS_RX" | "ABS_RY":
                return event.state / 32768.0
            case "ABS_RZ" | "ABS_Z":
                return event.state / 1024.0
            case _:
                return event.state