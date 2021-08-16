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
the implementation of forward and inverse kinematics.
- The direction (x, y or z axis) and length of the test movement can be configured.

### Running the Mirai Sandbox tool
```bash
usage: mirai-sandbox [-h] [-f FREQUENCY] [-s TCP_SPEED] [-d DIMENSION]
                     [-l LENGTH] [-ip IP] [-j JOINT_TOLERANCE]
                     [-t TCP_TOLERANCE] [-v]
                     path [robot]

Micropsi Industries Robot SDK Tool

positional arguments:
  path                  Path to the Robot implementation
  robot                 Name of the robot model as defined in the implementation.

optional arguments:
  -h, --help            show this help message and exit
  -f FREQUENCY, --frequency FREQUENCY
                        Frequency of the robot control loop, Hertz.
                        Default: 20
  -s TCP_SPEED, --tcp_speed TCP_SPEED
                        TCP speed, meters per second.
                        Default: 0.05, Max: 0.1
  -d DIMENSION, --dimension DIMENSION
                        Number of axes to move the robot in.
                        Default: 1
  -l LENGTH, --length LENGTH
                        Length of test movement, meters.
                        Default:0.05, Max: 0.1m
  -ip IP, --ip IP       IP address of the robot.
                        Default: 192.168.100.100
  -j JOINT_TOLERANCE, --joint_tolerance JOINT_TOLERANCE
                        Accuracy of the robot joints,  (units determined by implementation).
                        Default: 0.01
  -t TCP_TOLERANCE, --tcp_tolerance TCP_TOLERANCE
                        Accuracy of the TCP position achieved by robot.
                        Default: 0.01 meters
  -v, --verbose         Enable traceback on errors.

Usage example: mirai-sandbox ./examples/myrobot
```