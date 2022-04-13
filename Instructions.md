# The Micropsi Integration SDK

## Introduction

The Micropsi Integration SDK enables the development of interfaces between Micropsi Industries’ AI-driven robot control product MIRAI and robot platforms with the goal of bringing MIRAI’s on-site trainable visual servoing capabilities to new robots.

This is done by implementing a Python 3.6 class inheriting an interface defined in the `JointPositionRobot` abstract base class. This implementation, the “Robot Implementation”, typically provides the means of communication with the robot (usually a protocol over TCP/IP or UDP) and the inverse and forward kinematics plus safety checks.

A sandbox tool is provided for testing the implementation of the `JointPositionRobot` class during development. After the `JointPositionRobot` class works well in the sandbox, it can be integrated into the released MIRAI product in a joint effort together with Micropsi Industries engineers.

## Preconditions

### Robot Platform Requirements

For a successful implementation to be possible, the robot platform to be supported must…

* an articulated robot, i.e. a kinematic chain with a fixed base on one end and a movable tool on the other end
* be able to execute cartesian displacements and/or rotations provided by MIRAI in the tool frame
* be able to receive a continuous stream of movement commands at at least 20Hz and smoothly execute these movement commands

No assumption needs to be made about where kinematic calculations, path planning, or any form of smoothing will be implemented. Lightweight scenarios in which the Robot Implementation essentially just implements the communication protocol with the robot, forwards MIRAI displacement commands and reports positions, are possible and preferred, but heavier implementations in which kinematic calculations and even path planning are performed as part of the Robot Implementation are possible and common. It’s important to remember however that most code in a Robot Implementation will be called as part of a high-frequency control loop and needs to execute fast.

### Software Development Requirements

The software environment needed to develop a Robot Implementation consists of:
* Physical access to the robot to be supported, in a safe experimental setup
* Optional: a faithful software simulation of the robot for first steps
* Linux or macOS
* Python 3.6
* Pip
* Virtualenv (recommended)

## Implementation Process

A Platform SDK-based development of a Robot Implementation follows a 4-phase implementation process:

1. **Development Setup**
Setting up a Python 3.6 environment with all dependencies provided by the MIRAI Sandbox. Should additional libraries be required, they can be added to the virtualenv, but should be version-pinned and documented and ideally discussed with Micropsi Industries to make sure a successful step 4 (MIRAI product integration) is possible. Additional operating-system level dependencies should be avoided if at all possible, but can of course always be discussed.
2. **Implementation**
Implementing all the methods in the JointPositionRobot ABC.
3. **Sandbox validation**
Run all tests provided as part of the MIRAI Sandbox and observe the physical robot’s behavior. If one of the tests fails, or all tests pass, but the robot does not physically behave as expected, go back to step 2. The MIRAI Sandbox is part of the Integration SDK and is essentially a lightweight “mock” MIRAI, behaving towards the Robot Implementation exactly like a fully-fledged MIRAI system would, but validating and profiling the Robot Implementation.
4. **MIRAI Integration**
Once a successful validation of the Robot Implementation has been achieved, a full product integration can be performed. This will be done as a joint effort between Integration SDK users and Micropsi Industries engineers.

## Architecture Overview and Definitions

The basic structure of a Sandbox development setup is this:

[image]

The Robot Implementation essentially functions as glue code between Python method calls made to the methods defined in the JointPositionRobot ABC and the native real time protocol the robot speaks. Depending on the robot platform’s capabilities, some calculations such as inverse kinematics may have to be performed in the Robot Implementation. The MIRAI Sandbox can be transparently replaced with the actual MIRAI system in a subsequent step.

## Setting up the Environment

1. Install Python 3.6
2. pip install micropsi-integration-sdk
3. Create a directory in which your implementation will live and cd to that directory
4. wget https://raw.githubusercontent.com/micropsi-industries/micropsi-integration-sdk/0.0.5/examples/myrobot/__init__.py
5. (Rename and) edit myrobot.py and start working on the implementation.

## Interface Documentation

The ABC for the robot implementation can be found here: https://github.com/micropsi-industries/micropsi-integration-sdk/blob/0.0.5/micropsi_integration_sdk/robot_sdk.py See the docstrings for information on the class and the individual methods to be implemented in your myrobot.py class.

## Validation

Validation (during development and before integration) is performed with the mirai-sandbox tool. It can be installed from the command line via:
```
pip install micropsi-integration-sdk
```
Robot installations can be tested by passing the name of the Python implementation file to the sandbox, for example, from the directory in which the implementation myrobot.py resides:
```
mirai-sandbox myrobot.py -ip 192.168.125.1
```
This will launch the sandbox, pass the IP address to the Robot implementation in foo.py, and call all methods in foo.py just like a full MIRAI runtime would. Instead of an AI-driven skill however, a simple 5cm back-and-forth-movement in the x dimension will be generated for execution on the robot. Once the movement is perfomed as expected, and the sandbox does not print any error messages, the implementation can be considered good and is ready for integration with the full MIRAI runtime.

The sandbox uses very defensive default settings. It is good practice to start with these settings when working on a physical robot. For platforms that offer a simulator, developing and validating the implementation against the simulator first is recommended. Once a safe and error-free execution with the defensive default parameters has been shown, a number of parameters are available to get more ambitious:

|Parameter|Explanation|
|---|---|
| -r --robot | Name of the robot model to pass to the implementation for cases in which an implementation supports multiple models. Defaults to the only model if only a single model is supported or to the first model if multiple models are supported.|
| -ip --ip | The IP address to be passed to the implementation. Defaults to 192.168.100.100 |
| -f --frequency  | The control loop frequency that the sandbox or MIRAI should try to achieve in Hz. The sandbox will try to update its information on the robot pose at this frequency and send target pose updates at this frequency. Defaults to 20Hz.|
| -s --speed  | Speed at which the robot TCP should move when performing the validation movement in meters/s. Defaults to 0.05m/s.|
| -d --dimension | Asks the sandbox to perform the validation movement in the given number of dimensions. Defaults to 1, the x dimension. Y and Z can be added by specifying 2 or 3 as a value. Rotation validation movements are not supported by the current version of the sandbox, but will be in the future.|
| -l --length | Distance the TCP should travel in each dimension when performing the validation movement in meters. Defaults to 0.05cm.|
| -j –-joint_tolerance | Accuracy expectation for the joint position validation performed by the sandbox. The sandbox sends motion commands to the implementation and expects the robot to faithfully execute these commands. The joint tolerance (in radians) defines the maximum allowed deviation between the reported joint positions and the expected joint positions. Defaults to 0.1|
| -t –-tcp_tolerance  | Accuracy expectation for the tcp position validation performed by the sandbox. The sandbox sends motion commands to the implementation and expects the robot to faithfully execute these commands. The tcp tolerance (in meters) defines the maximum allowed deviation between the reported tcp position and the expected tcp position. Defaults to 0.01m|
| -v --verbose  | Specifying –v will cause the sandbox to print full stack traces instead of just error messages when exceptions are caught originating inside the implementation.|

Document date: Aug 16th 2021
