from abc import ABC, abstractmethod
from collections import namedtuple
from typing import Optional

import numpy as np

HardwareState = namedtuple("HardwareState", (
    "joint_positions",
    "joint_speeds",
    "joint_temperatures",
    "raw_wrench",
))


class JointPositionRobot(ABC):
    """
    Methods to be implemented for controlling a robot.
    The implementation of this ABC will be used by the environment (usually the MicroPsi runtime)
    to execute robot skills.

    # Initialization
    Some of the methods in this class are called once before or during initialisation, to establish
    basic information and kinematic properties of the robot: get_model, get_joint_count,
    get_joint_speed_limits and get_joint_position_limits are examples of these.
    The information provided by these methods will be used for plausibility checks once the control
    loop is active.

    # State manipulation
    A second set of methods is called to inform the implementation of the environment's state and
    ask for state transition in the implementation. These are connect, disconnect,
    is_ready_for_control, take_control and release_control.
    Connect and disconnect will be called infrequently, asking the implementation to manage the
    communication state between it and the robot. Once a connection is established, the
    implementation should maintain a connected state and perform reconnects if connection drops
    occur.
    is_ready_for_control, take_control and release_control are called more frequently, at the
    beginning and end of each robot skill.

    # Realtime control
    A third set of methods is called between take_control and release_control, when the robot is
    being controlled through the implementation and executing the movements generated by the
    MicroPsi runtime. These are get_hardware_state, clear_cached_hardware_state, forward_kinematics,
    inverse_kinematics, are_joint_positions_safe and, finally, send_joint_positions.
    send_joint_positions is the call asking for actual execution of a tool displacement, the other
    calls are made in preparation of the parameters for send_joint_positions.
    All methods in this group will be called at relatively high frequency (commonly 50Hz) and the
    sum of their execution times therefore needs to fit into a 20ms time box.
    Note that send_joint_positions in particular cannot be blocking until the robot has executed
    the movement. The environment does not expect complete execution of the requested movement. It
    will not make any guarantees about executability (i.e. in terms of joint acceleration) of the
    requested movement on the actual hardware. The time passing in one execution cycle is called a
    "period" in the docstrings of these methods.

    # Non-realtime control
    A fourth set of methods is for non-real time control of the robot, usually triggered by user
    interaction: command_move and command_stop can be blocking and should reliably leave the robot
    in the requested state
    """

    def __init__(self, ip_address: str, model: str, frequency: float):
        self.__ip_address = ip_address
        self.__model = model
        self.__frequency = frequency

    @property
    def ip_address(self) -> str:
        """
        IP address at which the robot hardware may be reached.
        """
        return self.__ip_address

    @property
    def model(self) -> str:
        """
        Robot model name.
        """
        return self.__model

    @property
    def frequency(self) -> float:
        """
        The frequency (Hz) at which the environment (usually the MicroPsi runtime) will call the
        realtime control methods as discussed in the class docstring.
        """
        return self.__frequency

    ##################
    # Initialization #
    ##################

    @staticmethod
    @abstractmethod
    def get_supported_models() -> list:
        """
        Return a list of unique robot model names (str) supported by this class. Any of these names
        may be used to initialize instances of the class.
        """
        raise NotImplementedError

    @abstractmethod
    def get_joint_count(self) -> int:
        """
        Return the number of joints (degrees of freedom) of the robot.

        Examples:
            For a robot with 6 joints:
                return 6
        """
        raise NotImplementedError

    @abstractmethod
    def get_joint_speed_limits(self) -> np.array:
        """
        Return the speed limits of each joint in the robot in order from base to end-effector.

        Examples:
            For a robot with 6 revolute joints, that can each rotate at up to pi rad/s:
                return np.array([
                    np.pi,
                    np.pi,
                    np.pi,
                    np.pi,
                    np.pi,
                    np.pi,
                ])
        """
        raise NotImplementedError

    @abstractmethod
    def get_joint_position_limits(self) -> np.array:
        """
        Return the position limits of each joint in order from base to end-effector.
        Each limit should be returned as a pair of floats (min, max).

        Examples:
            For a robot with 6 revolute joints, that can each rotate one full turn in each
                direction:
                return np.array([
                    [-pi*2, pi*2],
                    [-pi*2, pi*2],
                    [-pi*2, pi*2],
                    [-pi*2, pi*2],
                    [-pi*2, pi*2],
                    [-pi*2, pi*2]
                ])
        """
        raise NotImplementedError

    #######################
    # State manipulation. #
    #######################

    @abstractmethod
    def connect(self) -> bool:
        """
        Set up communication interfaces and connect to the hardware.
        Must return True if communication is successfully initialized.
        """
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        """
        Disconnect from the hardware and tear down communication interfaces.
        """
        raise NotImplementedError

    @abstractmethod
    def prepare_for_control(self) -> None:
        """
        Perform initialization steps necessary to make the robot ready to start receiving
        joint positions in realtime.
        """
        raise NotImplementedError

    @abstractmethod
    def is_ready_for_control(self) -> bool:
        """
        The MicroPsi runtime will call this to test whether the robot is ready to accept joint
        positions in realtime.
        Returning False from this method may cause the MicroPsi runtime to call prepare_for_control.
        If no setup step is required for realtime control, this method should return True.
        """
        raise NotImplementedError

    @abstractmethod
    def take_control(self) -> None:
        """
        Inform the robot that the MicroPsi runtime will start sending joint positions in realtime,
        and the robot should assume these positions as soon as they're received.
        """
        raise NotImplementedError

    @abstractmethod
    def release_control(self) -> None:
        """
        Inform the robot that the controller will not send any updated joint positions until the
        next time control is taken.
        """
        raise NotImplementedError

    ####################
    # Realtime control #
    ####################

    @abstractmethod
    def get_hardware_state(self) -> Optional[HardwareState]:
        """
        Return state information from the robot platform as a HardwareState object.

        joint_positions and joint_speeds should be populated in units appropriate to the robot
        platform. Units should be consistent between those returned from this get_hardware_state
        function, and those expected as input to the forward_kinematics function.
        raw_wrench (if available) should be returned relative to the end-effector frame, and
        should be a 6-element numpy array in Newtons and Newton-Meters:
            array([Fx, Fy, Fz, Tx, Ty, Tz])

        Notes:
            1. If the implementation cannot produce a valid hardwareState object, it should
                instead return None. Returning incorrect or partial values may cause the MicroPsi
                runtime to issue a stop command.
            2. The exceptions to point 1. are the optional joint_temperatures and raw_wrench fields.
                If a robot platform cannot provide raw wrench or joint temperature data natively,
                these fields should each be set to None in an otherwise valid HardwareState
                object.
        """
        raise NotImplementedError

    @abstractmethod
    def clear_cached_hardware_state(self) -> None:
        """
        Clear any hardware state information that may have been cached between calls to
        get_hardware_state().
        The MicroPsi runtime will call this method when it needs to be 100% certain of the freshness
        and accuracy of the returned state information on the next call to get_hardware_state().
        """
        raise NotImplementedError

    @abstractmethod
    def forward_kinematics(self, *, joint_positions: np.array) -> np.array:
        """
        Return the end-effector pose in {BASE} frame for the provided joint positions,
        as a 4x4 homogeneous transform matrix:
            R11 R12 R13 Dx
            R21 R22 R23 Dy
            R31 R32 R33 Dz
            0   0   0   1
        """
        raise NotImplementedError

    @abstractmethod
    def inverse_kinematics(self, *, end_effector_pose: np.array,
                           joint_reference: Optional[np.array]) -> Optional[np.array]:
        """
        Return the joint positions required to achieve the provided end_effector_pose.
        If no acceptable solution can be found, return None.

        Args:
            joint_reference (if not None): a nearby joint configuration to be used when
                choosing the best solution.
        """
        raise NotImplementedError

    @abstractmethod
    def are_joint_positions_safe(self, *, joint_positions: np.array) -> bool:
        """
        Return True if the provided joint positions represent a pose that is safe from self- or
        environment-collisions, and close enough to the current joint positions that they could be
        safely achieved within a single period at the configured frequency.
        """
        raise NotImplementedError

    @abstractmethod
    def send_joint_positions(self, *, joint_positions: np.array, period: float,
                             step_count: int) -> None:
        """
        Send joint positions to the robot for immediate execution.
        Another call to this method can be expected after the period has elapsed, so the hardware
        should achieve the provided joint positions within a single period in order to be ready
        for the next instruction.
        """
        raise NotImplementedError

    ########################
    # Non-realtime control #
    ########################

    @abstractmethod
    def command_move(self, *, joint_positions: np.array) -> None:
        """
        Send a move command to the robot.
        This is NOT a realtime command.
        It is likely that the joint positions provided will differ significantly from the current
        joint positions.
        The implementation is expected to handle all soft-start/stop behaviour, and can take as
        long as necessary to achieve the provided joint positions safely.

        Args:
            joint_positions: Numpy array of target joint positions that should eventually be
                achieved.
        """
        raise NotImplementedError

    @abstractmethod
    def command_stop(self) -> None:
        """
        Send a stop command to the robot.
        This method may be called in safety-critical situations, so it's important that all
        movement is immediately halted.
        """
        raise NotImplementedError

    @staticmethod
    def has_internal_ft_sensor() -> bool:
        """
        Optional, override as appropriate.

        Return whether the robot interface can provide FT data in {end-effector} frame natively.
        """
        return False
