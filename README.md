# es410-control-system
Control system for fighting robots project.

Required gamepad is xbox360 or xboxone

## Setup

use python 3.11 \
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
- subscribes to gamepad input reader
  
gamepad input reader
- read gamepad, convert to control outputs: L motor, R motor, Weapon motor
- publishes to main control
- output: list of (float[0, 1])
  
camera ML model (maybe separate program, running on diff machine)
- read camera, draw box around robots
  
camera robot identifier
- read camera, find QR codes and give info on where robots are
- output: (x: float, y: float, rot: float)
  
robot communication
- send and receieve data from robot
  
sensor data analysis
- analyse data received from robot, send to diff machine running camera ML model?

## Control Decisions

When user presses specific buttons on a gamepad, the system takes over to execute an automated function, e.g. drive towards other robot or similar.
