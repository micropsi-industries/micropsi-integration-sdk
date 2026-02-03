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
from micropsi_integration_sdk.sandbox_motion_generation import get_step_towards_goal
from micropsi_integration_sdk import robot_sdk
from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection

LOG = logging.getLogger(__name__)

DEFAULT_IP = "192.168.100.100"

DEFAULT_EE_SPEED_LINEAR = 0.05  # m/s
DEFAULT_EE_SPEED_ROT_DEGREES = 5.0  # °/s

DEFAULT_ACC_LINEAR = 0.001 # m
DEFAULT_ACC_ROT_DEGREES = 0.5 # °

DEF_MAX_DISTANCE_TRANSLATION = 0.05  # meters
DEF_MAX_DISTANCE_ROT_DEGREES = 10.0  # °


class RobotCommunication(threading.Thread):
    """
    Connection thread to continuously fetch the robot state
    """

    def __init__(self, *, robot_interface: robot_sdk.RobotInterface, tolerance_linear,
                 tolerance_rot_degrees, speed_limit_linear, speed_limit_rot):
        super().__init__(name="RobotCommunication", daemon=True)
        self.__frequency = robot_interface.get_frequency()
        self.__step_count = 0
        self.__interface: robot_sdk.RobotInterface = robot_interface
        self.__max_linear_step = speed_limit_linear / self.__frequency
        self.__max_rotation_step = speed_limit_rot / self.__frequency
        self.__end_effector_accuracy_linear = tolerance_linear
        self.__end_effector_accuracy_rot_degrees = tolerance_rot_degrees
        self.__slowdown_steps = (
            self.__interface.get_slowdown_steps_in_seconds() * self.__frequency
        )  # TODO verify
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
                self.update_state()
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

    def update_state(self):
        self.__state = self.get_state()
        self.__current_pose = (
            self.state.end_effector_pose if self.state.end_effector_pose is not None
            else self.__interface.forward_kinematics(joint_positions=self.state.joint_positions)
        )
        self.__state_received.set()

    def step(self):
        self.__step_count += 1

        if self.__goal_pose is None:
            # nowhere to go
            return

        if self.__at_goal():
            self.__goal_reached.set()
            return

        # compute one incremental pose change towards the goal, in
        # translation (3D array) and rotation (pyquaternion.Quaternion).
        # This is a delta, i.e. relative to current pose, expressed in base.
        linear_step, rotation_step_quat = get_step_towards_goal(
            current_xyz = self.current_pose[:3,3],
            current_q = Quaternion(matrix=self.current_pose[:3, :3]),
            goal_xyz = self.__goal_pose[:3,3],
            goal_q = Quaternion(matrix=self.__goal_pose[:3, :3]),
            stepsize_xyz = self.__max_linear_step,
            stepsize_rot = self.__max_rotation_step,
            slowdown_steps= self.__slowdown_steps,
        )

        # apply delta
        step_goal = np.identity(4)
        step_goal[:3, :3] = rotation_step_quat.rotation_matrix @ self.current_pose[:3, :3]
        step_goal[:3, 3] = self.current_pose[:3, 3] + linear_step

        if isinstance(self.__interface, robot_sdk.CartesianPoseRobot):
            self.__interface.send_goal_pose(goal_pose=step_goal, step_count=self.__step_count)

        elif isinstance(self.__interface, robot_sdk.CartesianVelocityRobot):
            linear_velocity = linear_step * self.__frequency
            angular_velocity = rotation_step_quat.axis * rotation_step_quat.radians * self.__frequency
            cartesian_velocity = np.concatenate([linear_velocity, angular_velocity])
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


    def set_action(self, *, action: np.ndarray):
        self.__goal_pose = self.current_pose @ action
        LOG.debug(f"=====================")
        LOG.debug(f"Current pose:\n{self.current_pose}")
        LOG.debug(f"Action:\n{action}")
        LOG.debug(f"New goal:\n{self.__goal_pose}")
        LOG.debug(f"=====================")
        self.wait_for_state()

    def __at_goal(self) -> bool:
        goal_pose = np.copy(self.__goal_pose)
        current_translate = self.current_pose[:3, 3]
        goal_translate = goal_pose[:3, 3]
        linear_distance = np.linalg.norm(goal_translate - current_translate)
        current_rotate = Quaternion(matrix=self.current_pose[:3, :3])
        goal_rotate = Quaternion(matrix=goal_pose[:3, :3])
        rotation_distance = (current_rotate.conjugate * goal_rotate).degrees
        LOG.debug("Linear distance: %.2fm", linear_distance)
        LOG.debug("Rotation distance: %.2f°", rotation_distance)
        return (abs(linear_distance) < self.__end_effector_accuracy_linear
                and abs(rotation_distance) < self.__end_effector_accuracy_rot_degrees)

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



def parse_args(args=None):
    parser = ArgumentParser(description="""
This will attempt to read poses and execute movements on a given robot interface.

Expected outcome: The robot moves through the following sequence of waypoints,
and ends up back at the start pose:

    # translations:
    1. Translate in tool +X by a set amount, then -X by the same amount (= return to origin)
    2. Similar for tool +/- Y
    3. Similar for tool +/- Z
    # single rotations:
    3. Rotate around tool +X by a set amount, then -X (= return to origin)
    4. Similar for tool +/- Y
    5. Similar for tool +/- Z
    5. Rotate around the tool XY diagnonal, then return
    6. Similar for the YZ diagonal
    7. Similar for the XZ diagonal
    # chained rotations:
    8. Rotate around tool Z, then Y, then X; then return in the reverse order.

See --help for config options (e.g. range of motion, speed, including/excluding some of the motions)
""",
                            epilog='Usage example: %s ../examples/cartesian_velocity_robot.py'
                                   % os.path.basename(sys.argv[0]),
                            formatter_class=RawTextHelpFormatter)

    parser.add_argument("path",
                        help="Path to the robot implementation")

    parser.add_argument("-m", "--model", type=str,
                        help="Name of the robot model as defined in the implementation.")

    parser.add_argument("-ip", "--ip-address", default=DEFAULT_IP, type=str,
                        help=f"IP address of the robot.\n"
                             f"Default: {DEFAULT_IP}")

    parser.add_argument("-sl", "--speed-linear", default=DEFAULT_EE_SPEED_LINEAR, type=float,
                        help=f"Linear end-effector speed, meters per second.\n"
                             f"Default: {DEFAULT_EE_SPEED_LINEAR}")

    parser.add_argument("-sa", "--speed-rotation", default=DEFAULT_EE_SPEED_ROT_DEGREES, type=float,
                        help=f"Rotational end-effector speed, degrees per second.\n"
                             f"Default: {DEFAULT_EE_SPEED_ROT_DEGREES}°")

    parser.add_argument("--test", nargs="*", default=None,
                        choices=["translations", "single-rotations", "chained-rotations"],
                        help="Select which tests to run, can specify one or multiple.\n"
                             "Choices: translations, single-rotations, chained-rotations.\n"
                             "Default: run all.")

    parser.add_argument("--max-distance-translation", default=DEF_MAX_DISTANCE_TRANSLATION, type=float,
                        help=f"Maximum distance for translational movements, meters.\n"
                             f"Default: {DEF_MAX_DISTANCE_TRANSLATION}")

    parser.add_argument("--max-distance-degrees", default=DEF_MAX_DISTANCE_ROT_DEGREES, type=float,
                        help=f"Maximum distance for rotational movements, degrees.\n"
                             f"Default: {DEF_MAX_DISTANCE_ROT_DEGREES}°")

    parser.add_argument("-tl", "--tolerance-linear", default=DEFAULT_ACC_LINEAR, type=float,
                        help=f"Consider a position linearly reached when within this distance, meters\n"
                             f"Default: {DEFAULT_ACC_LINEAR}m")

    parser.add_argument("-ta", "--tolerance-rotation", default=DEFAULT_ACC_ROT_DEGREES, type=float,
                        help=f"Consider a position rotationally reached when within this distance, degrees\n"
                             f"Default: {DEFAULT_ACC_ROT_DEGREES}°")

    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable debug logging.")

    parser.add_argument("-d", "--dimension", type=int, default=None,
                        help="DEPRECATED: See --test")
    parser.add_argument("-l", "--length", type=float, default=None,
                        help="DEPRECATED: See --max-distance-translation and --max-distance-degrees")

    return parser.parse_args(args=args)


def _check_deprecated_arguments(args):
    if args.dimension is not None:
        print(
            "  --dimension / -d is deprecated.\n"
            "    Use --test to select which tests to run:\n"
            "      --test translations\n"
            "      --test single-rotations\n"
            "      --test chained-rotations\n"
            "    By default, translations and single-rotations are run."
        )
        sys.exit(1)

    if args.length is not None:
        print(
            "  --length / -l is deprecated.\n"
            "    Use --max-distance-translation to set the maximum translation distance (meters),\n"
            "    and --max-distance-degrees to set the maximum rotational movement distance (degrees)."
        )
        sys.exit(1)

def main(args=None):
    if args is None:
        # We're running from the command line: args just come from the shell
        args = parse_args()
    else:
        # We're running from e.g. pytest: Args are passed in by our caller
        defaults = parse_args(['dummypath'])
        for key, value in vars(args).items():
            setattr(defaults, key, value)
        args = defaults

    _check_deprecated_arguments(args)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    path = args.path
    robot_model = args.model
    robot_ip = args.ip_address

    # Determine which tests to run
    if args.test is None or len(args.test) == 0:
        # Default: run all tests
        tests_to_run = {"translations", "single-rotations", "chained-rotations"}
    else:
        tests_to_run = set(args.test)

    max_distance_translation = args.max_distance_translation
    max_distance_degrees = args.max_distance_degrees

    tolerance_linear = args.tolerance_linear
    tolerance_rot_degrees = args.tolerance_rotation
    speed_limit_linear = args.speed_linear
    speed_limit_rot =args.speed_rotation

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
                                        tolerance_rot_degrees=tolerance_rot_degrees,
                                        speed_limit_linear=speed_limit_linear,
                                        speed_limit_rot=speed_limit_rot)

        preflight_checks(interface=interface, controller=controller)

        with controlled(interface):
            pose_changes = []
            descriptions = []

            if "translations" in tests_to_run:
                actions, descs = toolbox.generate_translations(max_distance_translation)
                pose_changes.extend(actions)
                descriptions.extend(descs)

            if "single-rotations" in tests_to_run:
                actions, descs = toolbox.generate_single_rotations(max_distance_degrees)
                pose_changes.extend(actions)
                descriptions.extend(descs)

            if "chained-rotations" in tests_to_run:
                actions, descs = toolbox.generate_chained_rotations(max_distance_degrees)
                pose_changes.extend(actions)
                descriptions.extend(descs)

            total = len(pose_changes)
            for idx, (pose_change, desc) in enumerate(zip(pose_changes, descriptions)):
                LOG.info(f"Testing movement {idx + 1}/{total}: {desc}")
                controller.set_action(action=pose_change)
                if not controller.wait_for_goal():
                    raise RuntimeError("Timed out while waiting for the robot to achieve the goal.")


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
