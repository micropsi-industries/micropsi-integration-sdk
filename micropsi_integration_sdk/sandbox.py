import os.path
import sys
import threading
import time
from argparse import ArgumentParser, RawTextHelpFormatter
from math import ceil

import numpy as np

import micropsi_integration_sdk.toolbox as toolbox
from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection
from micropsi_integration_sdk.toolbox import check_tcp_target, check_jnt_target

DEFAULT_IP = "192.168.100.100"
MAX_TCP_SPEED = 0.1
DEFAULT_TCP_SPEED = 0.05

DEFAULT_ACC = 1e-2
ACCURACY_MAX = 0.1

MAX_LINEAR_MOVEMENT = 0.1
DEF_FREQUENCY = 20

DEF_LENGTH = 0.05
MAX_LENGTH = 0.1

DEF_DIMENSION = 1

TIMEOUT = 5


class RobotCommunication(threading.Thread):
    """
    Connection thread to continuously fetch the robot state
    """

    def __init__(self, robot_interface, frequency):
        super().__init__(name="RobotCommunication")
        self.rob = robot_interface
        self.frequency = frequency
        self.state = None
        self.running = True
        self.step = 0
        self.last_target = None
        self.last_target_tcp = None

        self.thread_error = None
        self.logs = []
        self.last_flush = 0
        self.robot_connected = False

    def run(self):
        try:

            while self.running:
                start = time.time()

                self.state = self.get_state()

                elapsed = (time.time() - start)
                overstep = (1 / self.frequency) - elapsed

                if overstep > 0:
                    time.sleep(overstep)
                else:
                    self.add_log("WARNING: Robot frequency too high")

                secs_since_last_flush = time.time() - self.last_flush

                if secs_since_last_flush > 2:
                    self.flush_logs()
                    self.last_flush = time.time()

        except Exception as e:
            self.running = False
            self.thread_error = e

    def add_log(self, txt):
        if txt not in self.logs:
            self.logs.append(txt)

    def flush_logs(self):
        for i in self.logs:
            print(i)
        self.logs = []

    def move_joints(self, jnt_0, jnt_f, tcp_0, tcp_f, speed_lim_jnt,
                    speed_lim_tcp, tcp_accuracy, jnt_accuracy):
        """
        Moves the robot to target joint positions, by breaking down the delta
        into smaller deltas to sync with the frequency of robot communication.

        """
        assert jnt_f is not None
        assert jnt_0 is not None

        dist = toolbox.dist(tcp_0[:3], tcp_f[:3])

        jnt_diff = jnt_f - jnt_0

        vel = jnt_diff * self.frequency
        max_vel_ovrshoot = max(vel - speed_lim_jnt)
        delta_jnt = max(ceil(max_vel_ovrshoot), 1)

        delta = max(dist * self.frequency / speed_lim_tcp, delta_jnt)

        jnt_delta = jnt_diff / delta

        jnt_ = jnt_0.copy()
        for i in range(max(int(ceil(delta)), 1)):
            start = time.time()
            jnt_ = jnt_ + jnt_delta
            jnt_curr, tcp_curr = self.send_joint_positions(jnt_)
            elapsed = (time.time() - start)
            overstep = (1 / self.frequency) - elapsed
            time.sleep(max(overstep, 0))

        start = time.time()
        while not (check_jnt_target(jnt_, jnt_curr, jnt_accuracy)[0] and
                   check_tcp_target(tcp_f, tcp_curr, tcp_accuracy)[0]
        ) and self.running:
            jnt_curr, tcp_curr = self.send_joint_positions(jnt_)
            if time.time() - start > TIMEOUT:
                break

        jnt_c, tcp_c = self.manual_step()
        eq, msg = check_jnt_target(jnt_f, jnt_c, jnt_accuracy)
        assert eq, msg

        self.last_target = np.array(jnt_f)
        self.last_target_tcp = np.array(tcp_f)

    def move_robot(self, action, **kwargs):
        """
        Computes target joint values from actions and sends it to robot
        """

        jnt_acc = kwargs.get("jnt_accuracy")
        tcp_acc = kwargs.get("tcp_accuracy")
        if self.last_target is None:
            jnt_0, tcp_0 = self.manual_step()
        else:
            jnt_0 = self.last_target
            tcp_0 = self.last_target_tcp
        jnt_0_1, tcp_0_1 = toolbox.get_modified_joints(self.rob, tcp_0,
                                                       jnt_0, trans=action)

        self.move_joints(jnt_0, jnt_0_1, tcp_0, tcp_0_1, **kwargs)

        jnt_1, tcp_1 = self.manual_step()

        eq, msg = check_jnt_target(jnt_0_1, jnt_1, jnt_acc)
        assert eq, msg
        eq, msg = check_tcp_target(tcp_0_1, tcp_1, tcp_acc)
        assert eq, msg

        return jnt_0, jnt_0_1, tcp_1

    def get_state(self):
        """
        Connect to the robot and read the state.
        """
        state = None
        cnt = 0
        while state is None:
            cnt += 1
            state = self.rob.get_hardware_state()
            if state is None:
                if cnt > 10:
                    raise InterruptedError("Invalid state recieved, check"
                                           " robot connection")

                self.rob.clear_cached_hardware_state()
                self.rob.connect()
        return state

    def manual_step(self):
        self.step += 1
        tf = self.rob.forward_kinematics(joint_positions=self.state.joint_positions)
        return self.state.joint_positions, toolbox.extract_tcp(tf)

    def send_joint_positions(self, jnt):
        if not self.rob.are_joint_positions_safe(joint_positions=jnt):
            raise RuntimeError("Robot interface reported target joint positions as unsafe during "
                               "movement.")
        self.rob.send_joint_positions(joint_positions=jnt, step_count=self.step)
        return self.manual_step()

    def disconnect_robot(self):
        # Release Control
        self.rob.release_control()
        self.rob.disconnect()

    def close(self):
        """
        Shutdowns Thread and close the connection to the robot
        """
        self.running = False
        if self.is_alive():
            self.join()


def parse_args():
    parser = ArgumentParser(description="Micropsi Industries Robot SDK Tool",
                            epilog='Usage example: %s ./examples/myrobot'
                                   % os.path.basename(sys.argv[0]),
                            formatter_class=RawTextHelpFormatter)

    parser.add_argument("path", help="Path to the robot implementation")
    parser.add_argument("robot", nargs='?', default=None,
                        help="Name of the robot model as defined in the implementation.")
    parser.add_argument("-f", "--frequency", default=DEF_FREQUENCY, type=float,
                        help="Frequency of the robot control loop, Hertz.\n"
                             "Default: {}".format(DEF_FREQUENCY))
    parser.add_argument("-s", "--tcp_speed", default=DEFAULT_TCP_SPEED, type=float,
                        help="TCP speed, meters per second.\n"
                             "Default: {}, Max: {}".format(DEFAULT_TCP_SPEED, MAX_TCP_SPEED))
    parser.add_argument("-d", "--dimension", default=DEF_DIMENSION, type=int,
                        help="Number of axes to move the robot in.\n"
                             "Default: {}".format(DEF_DIMENSION))
    parser.add_argument("-l", "--length", default=DEF_LENGTH, type=float,
                        help="Length of test movement, meters.\n"
                             "Default:{}, Max: {}m".format(DEF_LENGTH, MAX_LENGTH))
    parser.add_argument("-ip", "--ip", default=DEFAULT_IP, type=str,
                        help="IP address of the robot.\n"
                             "Default: {}".format(DEFAULT_IP))
    parser.add_argument("-j", "--joint_tolerance", default=DEFAULT_ACC, type=float,
                        help="Accuracy of the robot joints,  "
                             "(units determined by implementation).\n"
                             "Default: {}".format(DEFAULT_ACC))
    parser.add_argument("-t", "--tcp_tolerance", default=DEFAULT_ACC, type=float,
                        help="Accuracy of the TCP position achieved by robot.\n"
                             "Default: {} meters".format(DEFAULT_ACC))
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable traceback on errors.")
    return parser.parse_args()


def main():
    args = parse_args()
    debug = args.verbose
    path = args.path
    robot_model = args.robot
    robot_ip = args.ip
    robot_frequency = args.frequency

    dist = args.length if args.length <= MAX_LINEAR_MOVEMENT else MAX_LINEAR_MOVEMENT
    jnt_accuracy = args.joint_tolerance if args.joint_tolerance <= ACCURACY_MAX else ACCURACY_MAX
    tcp_accuracy = args.tcp_tolerance if args.tcp_tolerance <= ACCURACY_MAX else ACCURACY_MAX
    tcp_speed_lim = args.tcp_speed if args.tcp_speed <= MAX_TCP_SPEED else MAX_TCP_SPEED

    if 0 < args.dimension < 4:
        dimensions = args.dimension
    else:
        print("\nDimensions out of Range: {}. \nCurrently only 3 dimensions, "
              "translations in 'x', 'y' and 'z' supported.\nFalling back to "
              "1 dimension movement.\n".format(args.dimension))
        dimensions = 1

    robot_path = toolbox.extract_path(path)

    collection = RobotInterfaceCollection()
    collection.load_interface(robot_path)
    supported_robots = sorted(collection.list_robots())
    e_list = []

    if len(supported_robots) == 0:
        print("No robot implementation found.")
        exit()

    if robot_model is None:
        if len(supported_robots) > 1:
            robot_list = ["%d: %s" % (idx, name) for idx, name in enumerate(supported_robots)]
            print("Multiple robot implementations found.")
            print("Please select a robot model:\n"
                  "{}".format(os.linesep.join(robot_list)))
            robot_idx = int(input("Index [0-%d]: " % (len(robot_list)-1)))
        else:
            print("Robot implementation found: "
                  "{}".format(supported_robots[0]))
            robot_idx = 0
        robot_model = supported_robots[robot_idx]
        print("Loading {} in 2 seconds..".format(supported_robots[0]))
        time.sleep(2)

    try:
        supported_robots.index(robot_model)
    except ValueError as e:
        print("NotImplementedError: Unknown/unsupported robot model")
        exit()

    robot_interface = collection.get_robot_interface(robot_model)

    robot_kwargs = {
        "frequency": robot_frequency,
        "model": robot_model,
        "ip_address": robot_ip,
    }

    rob = robot_interface(**robot_kwargs)

    if rob is None:
        print("Failed to load robot implementation")
        exit()

    thread = RobotCommunication(rob, robot_frequency)

    if not rob.model is robot_model:
        print("Invalid robot model loaded")
        exit()

    print("Robot {} implementation loaded".format(rob.model))
    print("Connecting to robot '{}'".format(robot_model))
    try:
        assert rob.connect(), "Robot connection failed"
    except AssertionError as e:
        print("ConnectionError: robot connection failed.")

    try:
        thread.start()
        while thread.state is None and thread.running:
            time.sleep(0.1)
        print("Connected")
        jnt_speed_lim = rob.get_joint_speed_limits()
        assert thread.state is not None, "Invalid robot State"

        jnt_cnt = rob.get_joint_count()
        jnt_speed_lmt = rob.get_joint_speed_limits()
        jnt_pos_lmt = rob.get_joint_position_limits()
        has_internal_ft = rob.has_internal_ft_sensor()

        if has_internal_ft:
            err_txt = "Invalid FT data: {}".format(thread.state.raw_wrench)
            assert thread.state.raw_wrench is not None, err_txt
            assert len(thread.state.raw_wrench) == 6, err_txt
        else:
            err_txt = "raw_wrench is expected to be None if no internal FT "
            "sensor present. value found: {}".format(thread.state.raw_wrench)
            assert thread.state.raw_wrench is None, err_txt

        jnt_e = "Invalid joint positions"
        assert len(thread.state.joint_positions) == jnt_cnt, jnt_e
        assert len(jnt_pos_lmt) == jnt_cnt, jnt_e
        assert len(jnt_speed_lmt) == jnt_cnt, jnt_e

        rob.connect()

        rob.release_control()
        rob.prepare_for_control()

        jnt_0, tcp_0 = thread.manual_step()

        rob.release_control()
        rob.prepare_for_control()
        rob.take_control()

        kwargs = {
            "speed_lim_jnt": jnt_speed_lim,
            "speed_lim_tcp": tcp_speed_lim,
            "jnt_accuracy": jnt_accuracy,
            "tcp_accuracy": tcp_accuracy,
        }
        print("Moving in {} axes, with distance {}".format(dimensions, dist))

        # Send move to current position instruction to protect against
        # outdated move instructions in robot register.
        thread.move_joints(jnt_0, jnt_0, tcp_0, tcp_0, **kwargs)
        eq, msg = check_jnt_target(jnt_0, thread.state.joint_positions,
                                   jnt_accuracy)
        assert eq, msg

        actions = toolbox.gen_random_actions(dimensions, dist=dist)

        for i in range(len(actions)):
            print("Moving to: Position  {}".format(i + 1))
            j0, jf, tf = thread.move_robot(actions[i], **kwargs)
            time.sleep(2.)

        eq, msg = check_tcp_target(tf, tcp_0, tcp_accuracy)
        assert eq, msg
        eq, msg = check_jnt_target(jf, jnt_0, jnt_accuracy)
        assert eq, msg

        # Release Control
        rob.release_control()
        rob.disconnect()
        print("{} Disconnected".format(rob.model))

    except Exception as e:
        if not thread.running:
            if debug:
                raise thread.thread_error
            if thread.thread_error is None:
                e_list.append(RuntimeError("Robot communication failed"))
            else:
                e_list.append(thread.thread_error)
        e_list.append(e)

        if debug:
            raise

        for err in e_list:
            s = err.args[0] if err.args else ""
            err_txt = type(err).__name__ + ": " + s
            print(err_txt)

    finally:
        for action in [thread.close, rob.release_control, rob.disconnect]:
            try:
                action()
            except:
                pass


if __name__ == '__main__':
    main()
