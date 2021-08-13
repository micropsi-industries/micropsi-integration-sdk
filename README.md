# Micropsi Industries integration sdk
Package for implementing and testing robots to be inegrated with Mirai

## Installation
Package can be installed by
```bash
pip install .
```
## Robot SDK
```JointPositionRobot``` Interface provides list of methods that needs to be implemented for the 
robot control.


## Mirai Sandbox
Stand alone tool to test the Robot SDK based Robot control implementation.
- Moves the robot and verifies the implementation of methods described in Robot SDK. Most 
importantly the implementation of IK and FK.
- The direction (x, y or z axis) and length of the movement can be configured.

### Running the Mirai Sandbox tool

```sh
mirai-sandbox -r ${Robot-Model} -d 3 -p ${path to robot implementation} -ip="192.168.100.100"
```
  | Requried parameters | Description |
  | ------ | ------ |
  | -p, --path |        path to the Robot implementation |
  | -r, --robot |      Robot Model as defined in the implementation |
  
| Optional parameters | Description |
  | ------ | ------ |
  |  -f, --frequency |  Frequency of the Robot in Hz. Default: 20Hz. Maximum Frequency 50
  |  -sp, --tcp_speed | TCP speed in meter per second Default: 0.1, Max: 0.1
  |  -d, --dimension  | Number of Axis to move the robot in. Default: 3
  |  -l, --length     | Length of movement in meters, Default:0.05m, Max: 0.1m
  |  -ip, --ip        | IP address of the robot. Default: 192.168.100.100
  |  -j, --joint_accuracy |  Accuracy of the robot joints. Default: 0.01
  |  -t, --tcp_accuracy  | Accuracy of the TCP position achieved by robot.Default: 0.01
  |  -db, --debug          | (Flag) Enable traceback of failed tests.
