import gamepad_parser
from time import sleep

def main():
    parser = gamepad_parser.GamepadParser()
    while True:
        print(parser.motor_outputs)
        print(parser.autonomousWeapon)
        print(parser.autonomousMovement)
        sleep(0.1)

if __name__ == "__main__":
    main()