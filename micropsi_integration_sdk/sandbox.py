import logging
import os.path
import sys
import threading
import time
from argparse import ArgumentParser, RawTextHelpFormatter
from contextlib import contextmanager

import numpy as np
from pyquaternion import Quaternion

import micropsi_integration_sdk.toolbox as toolbox
from micropsi_integration_sdk import robot_sdk
from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection

LOG = logging.getLogger(__name__)

DEFAULT_IP = "192.168.100.100"
MAX_EE_SPEED = 0.1  # m/s
DEFAULT_EE_SPEED = 0.05  # m/s

MAX_EE_SPEED_ANGULAR = np.deg2rad(40)  # rad/s
DEFAULT_EE_SPEED_ANGULAR = np.deg2rad(15)  # rad/s

DEFAULT_ACC = 1e-3
DEFAULT_ACC_ANGULAR = 1e-2
ACCURACY_MAX = 0.1

MAX_LINEAR_MOVEMENT = 0.1

DEF_LENGTH = 0.05
MAX_LENGTH = 0.1

DEF_DIMENSION = 1

TIMEOUT = 5


class RobotCommunication(threading.Thread):
    """
    Connection thread to continuously fetch the robot state
    """

    def __init__(self, *, robot_interface: robot_sdk.RobotInterface, tolerance_linear,
                 tolerance_angular, speed_limit_linear, speed_limit_angular):
        super().__init__(name="RobotCommunication", daemon=True)
        self.__frequency = robot_interface.get_frequency()
        self.__step_count = 0
        self.__interface: robot_sdk.RobotInterface = robot_interface
        self.__max_linear_step = speed_limit_linear / self.__frequency
        self.__max_angular_step = speed_limit_angular / self.__frequency
        self.__end_effector_accuracy_linear = tolerance_linear
        self.__end_effector_accuracy_angular = tolerance_angular
        self.__goal_pose = None
        self.__running = True
        self.__state = None
        self.__current_pose = None

        # thread management
        self.__state_received = threading.Event()
        self.__goal_reached = threading.Event()
        self.start()

    @property
    def state(self):
        return self.__state

    @property
    def running(self):
        return self.__running

    @property
    def current_pose(self):
        return self.__current_pose

    def run(self):
        while self.__running:
            try:
                start = time.perf_counter()
                self.step()
                elapsed = time.perf_counter() - start
                remainder = (1 / self.__frequency) - elapsed
                if remainder > 0:
                    time.sleep(remainder)
                else:
                    LOG.warning("Robot stepping frequency not achieved.")
            except Exception as e:
                LOG.exception(e)
                self.__goal_pose = None
                self.__running = False

    def step(self):
        self.__step_count += 1
        self.__state = self.get_state()
        if self.current_pose is None:
            self.__current_pose = (
                self.state.end_effector_pose if self.state.end_effector_pose is not None
                else self.__interface.forward_kinematics(joint_positions=self.state.joint_positions)
            )
        self.__state_received.set()

        if self.__goal_pose is None:
            # nowhere to go
            return

        if self.__at_goal():
            self.__goal_reached.set()
            return

        displacement = toolbox.invert_transform(matrix=self.current_pose) @ self.__goal_pose
        linear_displacement = displacement[:3, 3]
        linear_distance = np.linalg.norm(linear_displacement)
        if linear_distance > 0:
            linear_direction = linear_displacement / linear_distance
            linear_step_displacement = min(self.__max_linear_step,
                                           linear_distance / 2)
            linear_step = linear_step_displacement * linear_direction
        else:
            linear_step = np.zeros(3)

        angular_displacement = Quaternion(matrix=displacement[:3, :3])
        angular_step_displacement = min(self.__max_angular_step,
                                        angular_displacement.radians / 2)
        if angular_step_displacement > 0:
            angular_step_quaternion = Quaternion(
                axis=angular_displacement.axis,
                radians=angular_step_displacement
            )
            angular_step = angular_step_quaternion.rotation_matrix
            angular_step_vector = angular_step_quaternion.axis * angular_step_displacement
        else:
            angular_step = np.identity(3)
            angular_step_vector = np.zeros(3)
        step = np.identity(4)
        step[:3, :3] = angular_step
        step[:3, 3] = linear_step
        step_goal = self.current_pose @ step
        # LOG.debug("Target pose:\n%s", step_goal)

        if isinstance(self.__interface, robot_sdk.CartesianPoseRobot):
            self.__interface.send_goal_pose(goal_pose=step_goal, step_count=self.__step_count)
        elif isinstance(self.__interface, robot_sdk.CartesianVelocityRobot):
            cartesian_velocity = np.hstack((linear_step, angular_step_vector)) * self.__frequency
            self.__interface.send_velocity(velocity=cartesian_velocity, step_count=self.__step_count)
        elif isinstance(self.__interface, robot_sdk.JointPositionRobot):
            joint_goal = self.__interface.inverse_kinematics(
                end_effector_pose=step_goal, joint_reference=self.state.joint_positions)
            if not self.__interface.are_joint_positions_safe(joint_positions=joint_goal):
                raise RuntimeError("Encountered unsafe joint_positions.")
            self.__interface.send_joint_positions(joint_positions=joint_goal,
                                                  step_count=self.__step_count)
        elif isinstance(self.__interface, robot_sdk.JointSpeedRobot):
            joint_goal = self.__interface.inverse_kinematics(
                end_effector_pose=step_goal, joint_reference=self.state.joint_positions)
            if not self.__interface.are_joint_positions_safe(joint_positions=joint_goal):
                raise RuntimeError("Encountered unsafe joint_positions.")
            joint_speeds = (joint_goal - self.state.joint_positions) * self.__frequency
            self.__interface.send_joint_speeds(joint_speeds=joint_speeds,
                                               step_count=self.__step_count)
        else:
            raise TypeError("Unsupported robot type %s" % type(self.__interface))

        # Use the last step goal as the current pose in the next step to ensure consistent step
        # chaining
        self.__current_pose = step_goal

    def set_action(self, *, action: np.ndarray):
        self.__goal_pose = self.current_pose @ action
        self.wait_for_state()

    def __at_goal(self) -> bool:
        goal_pose = np.copy(self.__goal_pose)
        current_translate = self.current_pose[:3, 3]
        goal_translate = goal_pose[:3, 3]
        linear_distance = np.linalg.norm(goal_translate - current_translate)
        current_rotate = Quaternion(matrix=self.current_pose[:3, :3])
        goal_rotate = Quaternion(matrix=goal_pose[:3, :3])
        angular_distance = (current_rotate.conjugate * goal_rotate).radians
        LOG.debug("linear distance: %.2f", linear_distance)
        LOG.debug("angular distance: %.2f", angular_distance)
        return (abs(linear_distance) < self.__end_effector_accuracy_linear
                and abs(angular_distance) < self.__end_effector_accuracy_angular)

    def get_state(self):
        """
        Connect to the robot and read the state.
        """
        state = None
        cnt = 0
        while state is None:
            cnt += 1
            state = self.__interface.get_hardware_state()
            if state is None:
                if cnt > 10:
                    raise InterruptedError("Invalid state recieved, check"
                                           " robot connection")

                self.__interface.clear_cached_hardware_state()
                self.__interface.connect()
        return state

    def close(self):
        """
        Shutdown thread and close the connection to the robot
        """
        self.__running = False
        if self.is_alive():
            self.join()

    def wait_for_goal(self, timeout=10):
        self.__goal_reached.clear()
        return self.__goal_reached.wait(timeout=timeout)

    def wait_for_state(self, timeout=1):
        self.__state_received.clear()
        return self.__state_received.wait(timeout=timeout)


def parse_args():
    parser = ArgumentParser(description="Micropsi Industries Robot SDK Tool",
                            epilog='Usage example: %s ./examples/cartesian_velocity_robot.py'
                                   % os.path.basename(sys.argv[0]),
                            formatter_class=RawTextHelpFormatter)

    parser.add_argument("path",
                        help="Path to the robot implementation")
    parser.add_argument("-m", "--model", type=str,
                        help="Name of the robot model as defined in the implementation.")
    parser.add_argument("-sl", "--speed-linear", default=DEFAULT_EE_SPEED, type=float,
                        help="Linear end-effector speed, meters per second.\n"
                             "Default: {}, Max: {}".format(DEFAULT_EE_SPEED, MAX_EE_SPEED))
    parser.add_argument("-sa", "--speed-angular", default=DEFAULT_EE_SPEED_ANGULAR, type=float,
                        help="Angular end-effector speed, radians per second.\n"
                             "Default: {}, Max: {}".format(DEFAULT_EE_SPEED_ANGULAR,
                                                           MAX_EE_SPEED_ANGULAR))
    parser.add_argument("-d", "--dimension", default=DEF_DIMENSION, type=int,
                        help="Number of axes to move the robot in.\n"
                             "Default: {}".format(DEF_DIMENSION))
    parser.add_argument("-l", "--length", default=DEF_LENGTH, type=float,
                        help="Length of test movement, meters.\n"
                             "Default:{}, Max: {}m".format(DEF_LENGTH,
                                                           MAX_LENGTH))
    parser.add_argument("-ip", "--ip-address", default=DEFAULT_IP, type=str,
                        help="IP address of the robot.\n"
                             "Default: {}".format(DEFAULT_IP))
    parser.add_argument("-tl", "--tolerance-linear", default=DEFAULT_ACC, type=float,
                        help="Linear tolerance of the end-effector position achieved by robot.\n"
                             "Default: {} meters".format(DEFAULT_ACC))
    parser.add_argument("-ta", "--tolerance-angular", default=DEFAULT_ACC, type=float,
                        help="Angular tolerance of the end-effector position achieved by robot.\n"
                             "Default: {} radians".format(DEFAULT_ACC_ANGULAR))
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable debug logging.")
    return parser.parse_args()


def main():
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    path = args.path
    robot_model = args.model
    robot_ip = args.ip_address

    dist = args.length if args.length <= MAX_LINEAR_MOVEMENT else MAX_LINEAR_MOVEMENT
    tolerance_linear = min(args.tolerance_linear, ACCURACY_MAX)
    tolerance_angular = min(args.tolerance_angular, ACCURACY_MAX)
    speed_limit_linear = min(args.speed_linear, MAX_EE_SPEED)
    speed_limit_angular = min(args.speed_angular, MAX_EE_SPEED_ANGULAR)

    if 0 < args.dimension < 4:
        dimensions = args.dimension
    else:
        LOG.warning("Dimensions out of Range: %d. Currently only <= 3 dimensions, translations in "
                    "'x', 'y' and 'z' supported. Falling back to 1 dimension movement.",
                    args.dimension)
        dimensions = 1

    robot_path = toolbox.extract_path(path)

    collection = RobotInterfaceCollection()
    collection.load_interface(robot_path)
    supported_robots = sorted(collection.list_robots())

    if len(supported_robots) == 0:
        exit("No robot implementation found.")

    if robot_model is None:
        if len(supported_robots) > 1:
            robot_list = ["%d: %s" % (idx, name) for idx, name in enumerate(supported_robots)]
            LOG.info("Multiple robot implementations found.")
            LOG.info("Please select a robot model:\n%s", os.linesep.join(robot_list))
            robot_idx = int(input("Index [0-%d]: " % (len(robot_list) - 1)))
        else:
            LOG.info("Robot implementation found: '%s'", supported_robots[0])
            robot_idx = 0
        robot_model = supported_robots[robot_idx]
    assert robot_model in supported_robots, "Unsupported robot model"

    LOG.info("Loading '%s'", robot_model)
    interface_class = collection.get_robot_interface(robot_model)
    interface = interface_class(model=robot_model, ip_address=robot_ip)

    with connected(interface):
        controller = RobotCommunication(robot_interface=interface,
                                        tolerance_linear=tolerance_linear,
                                        tolerance_angular=tolerance_angular,
                                        speed_limit_linear=speed_limit_linear,
                                        speed_limit_angular=speed_limit_angular)

        preflight_checks(interface=interface, controller=controller)

        with controlled(interface):
            LOG.info("Moving in %d axes, with distance %.2fm", dimensions, dist)
            movements = toolbox.generate_actions(dimensions=dimensions, distance=dist)
            for idx, movement in enumerate(movements):
                action = np.identity(4)
                action[:3, 3] = movement
                LOG.info("Moving to position %d", idx)
                controller.set_action(action=action)
                if not controller.wait_for_goal():
                    raise RuntimeError("Timed out while waiting for the robot to achieve the "
                                       "goal.")


@contextmanager
def connected(interface: robot_sdk.RobotInterface):
    try:
        LOG.info("Connecting")
        assert interface.connect() is True, "Failed to connect"
        yield
    finally:
        LOG.info("Disconnecting")
        interface.disconnect()


@contextmanager
def controlled(interface: robot_sdk.RobotInterface):
    try:
        LOG.info("Taking control")
        attempts = 0
        while not interface.is_ready_for_control():
            attempts += 1
            if attempts > 10:
                raise RuntimeError("Failed to prepare for control.")
            interface.prepare_for_control()
            time.sleep(.1)
        interface.take_control()
        yield
    finally:
        LOG.info("Releasing control")
        interface.release_control()


def preflight_checks(*, interface, controller):
    assert controller.wait_for_state(), "Failed to get initial state."
    if interface.has_internal_ft_sensor():
        err_txt = f"Invalid FT data: {controller.state.raw_wrench}"
        assert controller.state.raw_wrench is not None, err_txt
        assert len(controller.state.raw_wrench) == 6, err_txt
    else:
        assert controller.state.raw_wrench is None, (
            f"raw_wrench is expected to be None if no internal FT sensor present. "
            f"Value found: {controller.state.raw_wrench}")

    joint_error = "Invalid joint count"
    joint_count = interface.get_joint_count()
    assert len(controller.state.joint_positions) == joint_count, joint_error
    assert len(interface.get_joint_position_limits()) == joint_count, joint_error
    assert len(interface.get_joint_speed_limits()) == joint_count, joint_error


if __name__ == '__main__':
    main()
