# es410-control-system
Control system for fighting robots project.

Required gamepad is xbox360 or xboxone

## Setup

use python 3.11 or newer \
install tkinter (for linux, idk for windows): \
`sudo apt install python3-tk` \
clone from github\
make venv:\
`python3 -m venv venv`\
activate venv:\
`source venv/bin/activate`\
install a bunch of libraries:\
`python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu`\
`python3 -m pip install inputs ultralytics`\
should probably work after this with:\
`python3 main.py`

## Layout

main robot control script
- decision on what action to send to robot
- choose between manual and automatic control inputs
- reads from gamepad input reader
  
gamepad input reader
- read gamepad, convert to control outputs: L motor, R motor, Weapon motor
- have gamepad class that models current state, gamepad input updates new state
- output: left, right, weapon motor as float[-1, 1]
  
camera ML model (maybe separate program, running on diff machine)
- read camera, draw box around robots
  
camera robot identifier
- read camera, find QR codes and give info on where robots are
- output: (x: float, y: float, rot: float)
  
robot communication
- send and receieve data from robot
- read sensor data
  
sensor data analysis
- analyse data received from robot, send to diff machine running camera ML model?
- check no high temps, est. battery percentage, etc

## Control Decisions

When user presses specific buttons on a gamepad, the system takes over to execute an automated function, e.g. drive towards other robot or similar.

## Design ideas

- add a GUI maybe?
  - add sensor readings
  - warnings
  - battery level
- add input at start of what control system to use, i.e. if using a hammer or spinner or whatever
- where will auto weapon use be?
  - probably main.py so we can do both automatic and manual use (just sum them lol)