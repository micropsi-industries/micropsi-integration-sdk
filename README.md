# Micropsi Industries Integration SDK
Package for implementing and testing robots to be integrated with Mirai

## Installation
Package can be installed by
```bash
git clone git@github.com:micropsi-industries/micropsi-integration-sdk.git
cd ./micropsi-integration-sdk
pip3 install .
```

## Robot SDK
`JointPositionRobot` Abstract Interface declares the list of methods that must be implemented for
successful robot control.

## Mirai Sandbox
Stand alone tool to test the SDK-based Robot control implementation.
- Moves the robot and verifies the implementation of methods described in Robot SDK. In particular
the implementation of the high-frequency control loop.
- The direction (x, y or z axis) and length of the test movement can be configured.

### Running the Mirai Sandbox tool
```bash
usage: mirai-sandbox [-h] [-m MODEL] [-f FREQUENCY] [-sl SPEED_LINEAR]
                     [-sa SPEED_ANGULAR] [-d DIMENSION] [-l LENGTH]
                     [-ip IP_ADDRESS] [-tl TOLERANCE_LINEAR]
                     [-ta TOLERANCE_ANGULAR] [-v]
                     path

Micropsi Industries Robot SDK Tool

positional arguments:
  path                  Path to the robot implementation

optional arguments:
  -h, --help            show this help message and exit
  -m MODEL, --model MODEL
                        Name of the robot model as defined in the implementation.
  -f FREQUENCY, --frequency FREQUENCY
                        Frequency of the robot control loop, Hertz.
                        Default: 50
  -sl SPEED_LINEAR, --speed-linear SPEED_LINEAR
                        Linear end-effector speed, meters per second.
                        Default: 0.05, Max: 0.1
  -sa SPEED_ANGULAR, --speed-angular SPEED_ANGULAR
                        Angular end-effector speed, radians per second.
                        Default: 0.2617993877991494, Max: 0.6981317007977318
  -d DIMENSION, --dimension DIMENSION
                        Number of axes to move the robot in.
                        Default: 1
  -l LENGTH, --length LENGTH
                        Length of test movement, meters.
                        Default:0.05, Max: 0.1m
  -ip IP_ADDRESS, --ip-address IP_ADDRESS
                        IP address of the robot.
                        Default: 192.168.100.100
  -tl TOLERANCE_LINEAR, --tolerance-linear TOLERANCE_LINEAR
                        Linear tolerance of the end-effector position achieved by robot.
                        Default: 0.001 meters
  -ta TOLERANCE_ANGULAR, --tolerance-angular TOLERANCE_ANGULAR
                        Angular tolerance of the end-effector position achieved by robot.
                        Default: 0.01 radians
  -v, --verbose         Enable debug logging.

Usage example: mirai-sandbox ./examples/cartesian_robot.py
```