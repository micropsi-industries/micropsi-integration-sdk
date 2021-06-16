from abc import ABC, abstractmethod
from collections import namedtuple
from typing import Optional

import numpy as np

ServoRobotState = namedtuple("ServoRobotState", ("raw_wrench", "joint_positions",
                                                 "joint_speeds", "joint_temps"))


class ServoRobot(ABC):
    """
    Methods to be implemented in order to satisfy the interface of a servo-control robot.
    """

    def __init__(self, ip_address: str, model: str, frequency: float):
        import logging
        logger = logging.getLogger("world")
        logger.warning("SERVO ROBOT INIT")

    @staticmethod
    @abstractmethod
    def get_supported_models() -> list:
        """
        Return a list of unique robot model names (str) supported by this class. Any of these names
        may be used to initialize instances of the class in the mirai runtime.
        """
        raise NotImplementedError

    @abstractmethod
    def get_model(self) -> str:
        """
        Return the model name of the robot. Should be unique within a robot manufacturer.
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
        Each limit should be returned as a pair of values (min, max).

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
    def get_hardware_state(self) -> Optional[ServoRobotState]:
        """
        Return state information from configured hardware sources as a ServoRobotState object.

        Notes:
            1. If the implementation cannot produce a ServoRobotState object, it should
                return None. Returning incorrect values may cause the controller to issue a safety
                stop.
            2. The exception to point 1. is the raw_ft_data field. If an implementation cannot
                provide raw FT data natively, this field may be set to None. This will cause
                the control thread to collect data from a configured external FT sensor instead.
        """
        raise NotImplementedError

    @abstractmethod
    def clear_cached_hardware_state(self) -> None:
        """
        Clear any hardware state information that may have been cached between calls to
        get_hardware_state().
        The control thread will call this method when it needs to be 100% certain of the freshness
        and accuracy of the returned state information on the next call to get_hardware_state().
        """
        raise NotImplementedError

    @abstractmethod
    def forward_kinematics(self, *, joint_positions: np.array) -> np.array:
        """
        Return the end-effector pose in {BASE} frame, in the form of a 4x4 homogeneous transform.
        """
        raise NotImplementedError

    @abstractmethod
    def inverse_kinematics(self, *, end_effector_pose: np.array,
                           joint_reference: Optional[np.array]) -> Optional[np.array]:
        """
        Return the joint configuration required to achieve the provided end_effector_pose.
        joint_reference (if provided) is a nearby joint configuration to be used when
        choosing the best solution.
        If no acceptable solution can be found, return None.
        Do not return an invalid solution.
        """
        raise NotImplementedError

    @abstractmethod
    def check_collisions(self, *, joint_positions: np.array) -> bool:
        """
        Return True if the provided joint positions represent a pose that is safe from self- or
        environment- collisions.
        """
        raise NotImplementedError

    @abstractmethod
    def prepare_for_control(self) -> None:
        """
        Perform any initialization steps necessary to make the robot ready to start receiving
        joint positions.
        """
        raise NotImplementedError

    @abstractmethod
    def is_ready_for_control(self) -> bool:
        """
        Mirai will use this to determine whether the robot is ready to accept joint positions
        for servo control.
        Returning False from this method may cause mirai to call prepare_for_mirai_control().
        If no robot-side control script is required for real-time servo communication, return True.
        """
        raise NotImplementedError

    @abstractmethod
    def take_control(self) -> None:
        """
        Inform the robot or any running program that the controller will start sending absolute
        joint positions, and the robot should assume these positions as soon as they're received.
        """
        raise NotImplementedError

    @abstractmethod
    def release_control(self) -> None:
        """
        Inform the robot or any running program that the controller will not send any updated
        joint positions until the next time control is taken.
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

    @abstractmethod
    def command_move(self, *, joint_positions: np.array) -> None:
        """
        Send a move command to the robot.
        This is not a servo command to be achieved immediately, it is likely that the joint
        positions provided will differ significantly from the current joint positions.
        It is expected that the implementation will handle all soft-start/stop behaviour, and can
        take as long as it requires to achieve the requested positions safely.

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
