
import inputs

def read_gamepad_input():
    while True:
        events = inputs.get_gamepad()
        for event in events:
            # Process the event
            # You can access the event type, code, and state using event.ev_type, event.code, and event.state respectively
            # Add your code here to handle the gamepad input events
            print(event.ev_type, event.code, event.state)

# Call the function to start reading gamepad input
read_gamepad_input()
