# Micropsi Industries Integration SDK
Package for implementing and testing robots to be integrated with Mirai

## Sections:
- [Installation](#installation)
- [Python interfaces module](#python-interfaces-module)
- [Examples](#examples)
- [Mirai sandbox](#mirai-sandbox)
- [Mirai dev server](#mirai-dev-server)
- [Mirai dev client](#mirai-dev-client)

## Installation
Package can be installed by
```bash
git clone git@github.com:micropsi-industries/micropsi-integration-sdk.git
cd ./micropsi-integration-sdk
pip3 install .
```

## Python interfaces module
In the `micropsi_integration_sdk` python module can be found abstract interfaces declaring the
methods that must be implemented for successful control of each supported robot type.

## Examples
In the examples folder can be found toy examples of each robot implementation. These respond to the
sandbox and dev server tools and simulate simple robot motion, but do not communicate with real
hardware.
They can be used as a starting point when developing a new robot implementation.

## Mirai sandbox
Standalone tool to test the SDK-based Robot control implementation.
Moves the robot and verifies the implementation of methods described in Robot SDK. In particular
the implementation of the high-frequency control loop. 
The direction (x, y or z axis) and length of the test movement can be configured.
```bash
usage: mirai-sandbox [-h] [-m MODEL] [-sl SPEED_LINEAR] [-sa SPEED_ANGULAR]
                     [-d DIMENSION] [-l LENGTH] [-ip IP_ADDRESS]
                     [-tl TOLERANCE_LINEAR] [-ta TOLERANCE_ANGULAR] [-v]
                     path

Micropsi Industries Robot SDK Tool

positional arguments:
  path                  Path to the robot implementation

optional arguments:
  -h, --help            show this help message and exit
  -m MODEL, --model MODEL
                        Name of the robot model as defined in the implementation.
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

Usage example: mirai-sandbox ./examples/cartesian_velocity_robot.py
```
## Mirai dev server
This tool simulates a mirai controller in certain (very simplified) ways.
Once started, it listens on port 6599 for the commands sent by either your PLC or robot program,
or the [mirai-dev-client](#mirai-dev-client) tool.
It accepts commands as documented in the binary skill api.

**CAUTION**
When it receives the `ExecuteSkill` command, this tool will attempt to communicate with a robot
at the configured address, and will run through an approximation of a full skill execution.
If your sdk robot has been properly implemented, this should not produce any motion. It is strongly
recommended that you first test this in simulation before attempting to control real hardware.
```shell
usage: mirai-dev-server [-h] --robot-file ROBOT_FILE
                        [--robot-address ROBOT_ADDRESS]
                        [--server-address SERVER_ADDRESS] [--always-fail]

optional arguments:
  -h, --help            show this help message and exit
  --robot-file ROBOT_FILE
                        File where the sdk robot implementation can be loaded. 
                        eg: './examples/cartesian_velocity_robot.py' (default: None)
  --robot-address ROBOT_ADDRESS
                        Address where the mirai dev server can expect to find the robot,
                        for motion streaming. (default: localhost)
  --server-address SERVER_ADDRESS
                        Address that the mirai dev server should listen on. (default: 0.0.0.0)
  --always-fail         Cause the dev server to respond to every request with a failure message.
                        (default: False)

Usage example:
# mirai-dev-server --robot-file examples/cartesian_velocity_robot.py
```

### Mirai dev client
This tool functions as an example client. It can be used as an initial smoke-test to confirm the 
functionality of the `mirai-dev-server` tool, including that of the `ExecuteSkill` command.
Your goal as an integrator is to replicate the behaviour of this client tool in your own robot
or PLC program.

**CAUTION**
When it receives the `ExecuteSkill` command, the dev server will attempt to communicate with a 
robot at the configured address, and will run through an approximation of a full skill execution.
If your sdk robot has been properly implemented, this should not produce any motion. It is strongly
recommended that you first test this in simulation before attempting to control real hardware.
```shell
usage: mirai-dev-client [-h] [--server-address SERVER_ADDRESS]
                        {GetBoxMetadata,GetTrainedSkills,ExecuteSkill,PrepareSkillAsync,GetResult,
                         GetLastEndstateValues,GetExceptionMessage}

positional arguments:
  {GetBoxMetadata,GetTrainedSkills,ExecuteSkill,PrepareSkillAsync,GetResult,GetLastEndstateValues,
   GetExceptionMessage}

optional arguments:
  -h, --help            show this help message and exit
  --server-address SERVER_ADDRESS
                        Hostname or IP address where the mirai dev server is running. 
                        (default: localhost)

Usage example:
# mirai-dev-client GetBoxMetadata
```