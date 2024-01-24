from abc import ABC, abstractmethod
from collections import namedtuple
from typing import Optional

import numpy as np

HardwareState = namedtuple("HardwareState", (
    "joint_positions",
    "joint_speeds",
    "joint_temperatures",
    "raw_wrench",
    "end_effector_pose",
))
HardwareState.__new__.__defaults__ = (None,) * len(HardwareState._fields)


class RobotInterface(ABC):
    """
    Methods to be implemented for controlling a robot.
    The implementation of this ABC will be used by the environment (usually the MicroPsi runtime)
    to execute robot skills.

    # Initialization
    Some of the methods in this class are called once before or during initialisation, to establish
    basic information and kinematic properties of the robot: get_joint_count, get_joint_speed_limits
    and get_joint_position_limits are examples of these.
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
    MicroPsi runtime. These are get_hardware_state, clear_cached_hardware_state, and
    forward_kinematics. Specialization classes below have additional requirements in this group,
    as appropriate for their respective control-systems.
    All methods in this group will be called at relatively high frequency (commonly 50Hz) and the
    sum of their execution times therefore needs to fit into a single period at that frequency.
    The micropsi controller will not make any guarantees about executability (i.e. in terms of
    joint acceleration) of the requested movement on the actual hardware. The time passing in one
    execution cycle is called a "period" in the docstrings of these methods.

    # Non-realtime control
    A fourth set of methods is for non-real time control of the robot, usually triggered by user
    interaction: command_move and command_stop can be blocking and should reliably leave the robot
    in the requested state. In particular, command_stop may be called in safety-critical situations,
    so it's important that it is immediately executed on the hardware.
    """
    controller_type = "undefined"

    def __init__(self, ip_address: str, model: str):
        self.__ip_address = ip_address
        self.__model = model

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
    def get_joint_speed_limits(self) -> np.ndarray:
        """
        Return the speed limits of each joint in the robot in order from base to end-effector, in
        units per second appropriate to the robot platform.

        Examples:
            For a robot arm with 6 revolute joints, that can each rotate at up to pi rad/s:
                return array([pi, pi, pi, pi, pi, pi])

            For a robot gantry with 3 linear joints that can translate at up to 0.2 m/s
                and 1 revolute joint that can rotate at up to pi rad/s:
                return array([.2, .2, .2, pi])
        """
        raise NotImplementedError

    @abstractmethod
    def get_joint_position_limits(self) -> np.ndarray:
        """
        Return the position limits of each joint in order from base to end-effector.
        Each limit should be returned as a pair of floats (min, max), in units appropriate to the
        robot platform.

        Examples:
            For a robot arm with 6 revolute joints, that can each rotate one full turn in each
                direction:
                return array([[-pi*2, pi*2], [-pi*2, pi*2], [-pi*2, pi*2],
                              [-pi*2, pi*2], [-pi*2, pi*2], [-pi*2, pi*2]])

            For a robot gantry with 3 linear joints that can translate between -/+ 0.5m
                and 1 revolute joint that can rotate between -/+ pi radians:
                return array([[-.5, .5], [-.5, .5], [-.5, .5], [-pi, pi]])
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
    def forward_kinematics(self, *, joint_positions: np.ndarray) -> np.ndarray:
        """
        Return the end-effector pose in {BASE} frame for the provided joint positions,
        as a row-major, 4x4, homogeneous numpy array:
            array([[R11, R12, R13, Dx],
                   [R21, R22, R23, Dy],
                   [R31, R32, R33, Dz],
                   [0,   0,   0,   1]])
        """
        raise NotImplementedError

    @abstractmethod
    def are_joint_positions_safe(self, *, joint_positions: np.ndarray) -> bool:
        """
        Return True if the provided joint positions represent a pose that is safe from self- or
        environment-collisions.
        """
        raise NotImplementedError

    ########################
    # Non-realtime control #
    ########################

    @abstractmethod
    def command_move(self, *, joint_positions: np.ndarray) -> None:
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
        This method may be called in safety-critical situations, so it's important that all robot
        movement is immediately halted.
        """
        raise NotImplementedError

    @staticmethod
    def get_wait_for_receive_timeout() -> float:
        """
        Optional, override as appropriate.

        Return how long the mirai runtime should wait for receiving hardware state readings from the
        robot.
        """
        return 1.

    @staticmethod
    def has_internal_ft_sensor() -> bool:
        """
        Optional, override as appropriate.

        Return whether the robot interface can provide FT data in {end-effector} frame natively.
        """
        return False

    def get_frequency(self) -> float:
        """
        Optional, override as appropriate.

        The frequency (Hz) at which the environment (usually the MicroPsi runtime) may call the
        realtime control methods as discussed in the class docstring.
        Default is 50Hz. Override as appropriate.
        """
        return 50.

    def get_home_tolerance(self) -> float:
        """
        Optional, override as appropriate.

        When the robot is sent home, the home tolerance defines how much the sum of
        all current joint positions can differ from the sum of all joint position of
        the configured home position for the position to be considered "home". The
        unit for the joints is the same unit being returned by get_hardware_state.
        Default is 0.1.
        """
        return 0.1

    def get_sensitivity_range(self) -> dict:
        """
        Returns a dict in this form:
            {
                "force": (float, float),
                "torque": (float, float),
            }

        The float pairs are (min, max), and they control unit-less sensitivity parameters for force
        and torque response in the mirai control loop.

        Default values have been chosen, to suit currently supported platforms.
        Override as appropriate.
        """
        return {
            "force": (0.1, 0.7),
            "torque": (0.06, 0.22),
        }

    def get_slowdown_steps_in_seconds(self) -> float:
        """
        Optional, override as appropriate.

        Slowdown_steps is a parameter that defines how many steps the robot should need to reach
        the goal position. Using this parameter, the robot asymptotically slows down near the goal.
        This is to ensure a smooth approach without overshooting the goal position.

        Slowdown_steps needs to be adjusted according to a specific robot controller (based on its
        acceleration and decelaration values). Since the actual step length depends on the stepping
        frequency that can be adjusted later, the value is exposed in seconds.

        The default value is (11 / 15.15): (default slowdown_steps / default runtime frequency).
        """
        return (11 / 15.15)

    def get_virtual_dynamics_parameters(self) -> dict:
        """
        Optional, override as appropriate.

        Virtual dynamics parameters are used in calculating guiding motion inputs to translate FT
        sensor readings into TCP velocity. Returns:
            {
                "mass": float
                "moment": float
            }

        Mass is used to calculate acceleration for translational motion: Force readings / mass
        Moment is used to calculate acceleration for rotational motion: Torque readings / moment
        """
        return {
            "mass": 6.0,
            "moment": 0.04,
        }

    def get_guiding_filter_strength(self) -> float:
        """
        Optional, override as appropriate.

        Guiding filter strength is used to enable and smooth guiding motion inputs, i.e., FT
        sensor readings. It takes values between 0 and 1. The higher the value is the more smoothing
        applied on the readings to avoid a sudden increase on the forces applied. This, in the end,
        avoids sudden jumps on the robot caused by sudden changes in commanded velocities, which are
        calculated from the FT sensor readings.

        Additionally, the filter also subtracts the guiding threshold value (the value to beat to
        start guiding) from the readings. With that, we can now guide the robot with low velocities.

        By default, this is disabled.
        """
        return 0.0

    def get_ra_acceleration_values(self) -> dict:
        """
        Optional, override as appropriate.

        This function returns a dict of maximum acceleration values for the Recording Assistant (RA).
        RA plans a trajectory and executes motions on the robot, respecting these maximum acceleration
        values set. The values are in m/s^2 and deg/s^2 for linear and angular acceleration
        respectively. The values are used to limit the maximum linear and angular velocity changes
        between two steps of the controller. By doing so, we avoid sudden shifts in the robot's
        velocity, which could cause the robot to dangerously jerk and potentially overshoot the target
        positions. Hence, the values are specific to the robot's acceleration and deceleration
        capabilities.

        Default values below are tested to function well with URs (UR3, UR5, UR10) and Fanucs
        (CRX-10iA/L, LR Mate 200iD/7L).
        """
        return {
            "max_acceleration_cartesian": 0.01,
            "max_acceleration_degrees": 10,
            "max_deceleration_cartesian": 0.02,
            "max_deceleration_degrees": 60,
        }


class CartesianPoseRobot(RobotInterface):
    """
    Specialization of the RobotInterface class for communicating goal poses in cartesian
    coordinates. All methods of the base RobotInterface class must be implemented, and additionally
    the inheriting class must provide an implementation of the method(s) listed here.
    """
    controller_type = "cartesian_pose"

    ####################
    # Realtime control #
    ####################

    @abstractmethod
    def send_goal_pose(self, *, goal_pose: np.ndarray, step_count: int) -> None:
        """
        Send the goal end-effector pose to the robot for immediate execution.
        The goal will be provided as a 4x4 homogeneous transform relative to the base frame of the
        robot.
        Another call to this method can be expected after the period has elapsed, so the hardware
        should achieve the provided goal within a single period at the configured
        frequency in order to be ready for the next instruction.
        """
        raise NotImplementedError


# legacy class, provided for backwards-compatibility
class CartesianRobot(CartesianPoseRobot, ABC):
    controller_type = "cartesian"


class CartesianVelocityRobot(RobotInterface):
    """
    Specialization of the RobotInterface class for communicating goal velocities in cartesian
    (base frame) coordinates. All methods of the base RobotInterface class must be implemented, and
    additionally the inheriting class must provide an implementation of the method(s) listed
    here.
    """
    controller_type = "cartesian_velocity"

    ####################
    # Realtime control #
    ####################

    @abstractmethod
    def send_velocity(self, *, velocity: np.ndarray, step_count: int) -> None:
        """
        Send the goal end-effector velocity to the robot for immediate execution.
        The goal velocity will be provided as a 6D array relative to the base frame of the
        robot, in m/s and rad/s. [Vx, Vy, Vz, Wx, Wy, Wz]
        Notes:
            - The angular (W) components are provided in 3D axis-angle pseudo-vector form.

        Another call to this method can be expected after the period has elapsed, so the
        implementation should return within a single step at the configured frequency,
        regardless of whether the goal velocity has been reached or not.
        """
        raise NotImplementedError


class JointPositionRobot(RobotInterface):
    """
    Specialization of the RobotInterface class for communicating the goal poses as a joint_positions
    array. All methods of the base RobotInterface class must be implemented, and additionally
    the inheriting class must implement the method(s) declared here.
    """
    controller_type = "joint_position"

    ####################
    # Realtime control #
    ####################

    @abstractmethod
    def inverse_kinematics(self, *, end_effector_pose: np.ndarray,
                           joint_reference: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """
        Return the joint positions required to achieve the provided end_effector_pose.
        If no acceptable solution can be found, return None.

        Args:
            end_effector_pose: target pose for which joint positions should be computed.
            joint_reference (if not None): a nearby joint configuration to be used when
                choosing the best solution.
        """
        raise NotImplementedError

    @abstractmethod
    def send_joint_positions(self, *, joint_positions: np.ndarray, step_count: int) -> None:
        """
        Send the joint positions to the robot for immediate execution.
        The goal will be provided as a numpy array of appropriate length for the robot joint count.
        Another call to this method can be expected after the period has elapsed, so the hardware
        should achieve the provided joint positions within a single period at the configured
        frequency in order to be ready for the next instruction.
        """
        raise NotImplementedError


class JointSpeedRobot(RobotInterface):
    """
    Specialization of the RobotInterface class for communicating the goal poses as a joint_positions
    array. All methods of the base RobotInterface class must be implemented, and additionally
    the inheriting class must implement the method(s) declared here.
    """
    controller_type = "joint_speed"

    ####################
    # Realtime control #
    ####################

    @abstractmethod
    def inverse_kinematics(self, *, end_effector_pose: np.ndarray,
                           joint_reference: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """
        Return the joint positions required to achieve the provided end_effector_pose.
        If no acceptable solution can be found, return None.

        Args:
            end_effector_pose: target pose for which joint positions should be computed.
            joint_reference (if not None): a nearby joint configuration to be used when
                choosing the best solution.
        """
        raise NotImplementedError

    @abstractmethod
    def send_joint_speeds(self, *, joint_speeds: np.ndarray, step_count: int) -> None:
        """
        Send the joint speeds to the robot for immediate execution.
        The speeds will be provided as a 1D numpy array of appropriate length for the robot joint
        count.
        Another call to this method can be expected after the period has elapsed, so the hardware
        should achieve the provided joint speeds within a single period at the configured
        frequency in order to be ready for the next instruction.
        """
        raise NotImplementedError

    def use_recovery_locking(self) -> bool:
        """
        JointSpeedRobots tend to accumulate cartesian pose error.
        This becomes apparent when motion in certain axes is locked, because the robot will tend to
        drift away from the allowed pose into the locked direction and is generally unable to
        recover.
        Mirai can apply a recovery velocity to more closely align with locked axes, but this
        reduces the stepping frequency achievable by the controller, and so is disabled by default.

        Override this function to return True, if the recovery locking system should be enabled.
        """
        return False
