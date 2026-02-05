# Micropsi Industries Integration SDK
Package for implementing and testing robots to be integrated with Mirai.
A brief introduction and command reference can be found here.

For more detailed documentation, see [instructions.md](instructions.md)

For Skill API reference, see [skill_api.md](skill_api.md)

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
Standalone tool to test an SDK-based robot interface implementation. This will attempt
to read robot poses and send movement commands in ways similar to Mirai, helping to
uncover problems related to geometry and real time control.

If successful, the robot will go through a defined sequence of tool pose changes
and return to its start pose. See `--help` or below for details.

**CAUTION**
This will attempt to move the robot. Although this tool (by default) only
outputs low-speed commands, prepare for unexpected behavior including large,
high-speed movements when testing with a newly written interface.

```
usage: mirai-sandbox [-h] [-m MODEL] [-ip IP_ADDRESS] [-sl SPEED_LINEAR] [-sa SPEED_ROTATION] [--test [{translations,single-rotations,chained-rotations} ...]] [--max-distance-translation MAX_DISTANCE_TRANSLATION]
                     [--max-distance-degrees MAX_DISTANCE_DEGREES] [-tl TOLERANCE_LINEAR] [-ta TOLERANCE_ROTATION] [-v] [-d DIMENSION] [-l LENGTH]
                     path

This will attempt to read poses and execute movements on a given robot interface.

Expected outcome: The robot moves through the following sequence of waypoints,
and ends up back at the start pose:

    # translations:
    1. Translate in tool +X by a set amount, then -X by the same amount (= return to origin)
    2. Similar for tool +/- Y
    3. Similar for tool +/- Z
    # single rotations:
    4. Rotate around tool +X by a set amount, then -X (= return to origin)
    5. Similar for tool +/- Y
    6. Similar for tool +/- Z
    7. Rotate around the tool XY diagonal, then return
    8. Similar for the YZ diagonal
    9. Similar for the XZ diagonal
    # chained rotations:
    10. Rotate around tool Z, then Y, then X; then return in the reverse order.

See --help for config options (e.g. range of motion, speed, including/excluding some of the motions)

positional arguments:
  path                  Path to the robot implementation

options:
  -h, --help            show this help message and exit
  -m MODEL, --model MODEL
                        Name of the robot model as defined in the implementation.
  -ip IP_ADDRESS, --ip-address IP_ADDRESS
                        IP address of the robot.
                        Default: 192.168.100.100
  -sl SPEED_LINEAR, --speed-linear SPEED_LINEAR
                        Linear end-effector speed, meters per second.
                        Default: 0.05
  -sa SPEED_ROTATION, --speed-rotation SPEED_ROTATION
                        Rotational end-effector speed, degrees per second.
                        Default: 5.0°
  --test [{translations,single-rotations,chained-rotations} ...]
                        Select which tests to run, can specify one or multiple.
                        Choices: translations, single-rotations, chained-rotations.
                        Default: run all.
  --max-distance-translation MAX_DISTANCE_TRANSLATION
                        Maximum distance for translational movements, meters.
                        Default: 0.05
  --max-distance-degrees MAX_DISTANCE_DEGREES
                        Maximum distance for rotational movements, degrees.
                        Default: 10.0°
  -tl TOLERANCE_LINEAR, --tolerance-linear TOLERANCE_LINEAR
                        Consider a position linearly reached when within this distance, meters
                        Default: 0.001m
  -ta TOLERANCE_ROTATION, --tolerance-rotation TOLERANCE_ROTATION
                        Consider a position rotationally reached when within this distance, degrees
                        Default: 0.5°
  -v, --verbose         Enable debug logging.
  -d DIMENSION, --dimension DIMENSION
                        OBSOLETE: See --test
  -l LENGTH, --length LENGTH
                        OBSOLETE: See --max-distance-translation and --max-distance-degrees

Usage example: mirai-sandbox ../examples/cartesian_velocity_robot.py
```

## Mirai dev server
This tool simulates a mirai controller in certain (very simplified) ways.
Once started, it listens on port 6599 for the commands sent by either your PLC or robot program,
or the [mirai-dev-client](#mirai-dev-client) tool.
It accepts commands as documented in the binary skill api.

**CAUTION**
When it receives the `ExecuteSkill` command, this tool will attempt to communicate with a robot
at the configured address, and will run through an approximation of a full skill execution.
If your sdk robot has been properly implemented, this should produce only miniscule motion, however
this cannot be guaranteed.
It is strongly recommended that you first test this in simulation before attempting to interface 
with real hardware.
```shell
usage: mirai-dev-server [-h] --robot-file ROBOT_FILE
                        [--robot-address ROBOT_ADDRESS]
                        [--server-address SERVER_ADDRESS] [--always-fail]
                        [--keepalive KEEPALIVE]

optional arguments:
  -h, --help            show this help message and exit
  --robot-file ROBOT_FILE
                        File where the sdk robot implementation can be loaded. eg: './examples/cartesian_velocity_robot.py' (default: None)
  --robot-address ROBOT_ADDRESS
                        Address where the mirai dev server can expect to find the robot, for motion streaming. (default: localhost)
  --server-address SERVER_ADDRESS
                        Address that the mirai dev server should listen on. (default: 0.0.0.0)
  --always-fail         Cause the dev server to respond to every request with a failure message. (default: False)
  --keepalive KEEPALIVE
                        Keep idle client connections for KEEPALIVE seconds before dropping. (default: 60)

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
usage: mirai-dev-client [-h] [--server-address SERVER_ADDRESS] [--count COUNT]
                        [--period PERIOD] [--api-version {1,2}]
                        {GetBoxMetadata,GetTrainedSkills,ExecuteSkill,PrepareSkillAsync,GetResult,GetLastEndstateValues,GetExceptionMessage,KeepAlive}

positional arguments:
  {GetBoxMetadata,GetTrainedSkills,ExecuteSkill,PrepareSkillAsync,GetResult,GetLastEndstateValues,GetExceptionMessage,KeepAlive}

optional arguments:
  -h, --help            show this help message and exit
  --server-address SERVER_ADDRESS
                        Hostname or IP address where the mirai dev server is running. (default: localhost)
  --count COUNT         Send the command COUNT times, reusing the connection. (default: 1)
  --period PERIOD       Wait PERIOD seconds between sent messages. (default: 1)
  --api-version {1,2}   Format messages for the given mirai binary api version. (default: 1)

Usage example:
# mirai-dev-client GetBoxMetadata
```