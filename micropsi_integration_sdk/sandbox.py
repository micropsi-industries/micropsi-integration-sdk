from argparse import ArgumentParser

import os
import time
import threading
from math import ceil
import numpy as np
import micropsi_integration_sdk.ri_toolbox as rt
from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection


DEFAULT_IP = "192.168.100.100"
MAX_TCP_SPEED = 0.1
DEFAULT_TCP_SPEED = 0.1

DEFAULT_ACC = 1e-2
ACCURACY_MAX = 0.1

MAX_LINEAR_MOVEMENT = 0.1
MAX_FREQUENCY = 50
DEF_FREQUENCY = 20

LENGTH = 0.05
MAX_LENGTH = 0.1

THREAD = None
last_target = None
thread_stopped = True


def assert_wrapper(cond, txt):
    """
    Assert wrapper, with text describing the test
    """
    global THREAD
    try:
        if DEBUG:
            print("TESTING {}".format(txt))
        assert cond
    except Exception as e:
        try:
            THREAD.close()
        except:
            pass
        raise_wrapper(e, err_txt=txt)


def raise_wrapper(e, err_txt="", error_title=""):
    if e is not None:
        if len(e.args) > 0:
            s = e.args[0]
        else:
            s = ""
        error_title = type(e).__name__ + ": " + s

    print("{}. {}".format(error_title, err_txt))
    if DEBUG:
        raise
    exit()


def check_if_equal(arr_1, arr_2, E, pose=False, assrt=False):
    """
    Checks if two incomming poses are equivalent (delta < E).
    For pose only the linear component of position is verified
    """
    ret = True
    arr1 = arr_1.copy()
    arr2 = arr_2.copy()

    p = "TCP" if pose else "Joint"
    assert_wrapper(len(arr1) == len(arr2), "Invalid {} pose".format(p))

    if pose:
        arr1[3:6] = [0., 0., 0]
        arr2[3:6] = [0., 0., 0]

    for i in range(len(arr1)):
        delta = arr1[i] - arr2[i]
        if not abs(delta) < E:
            ret = False
            break
    if assrt:
        if pose:
            e_txt1 = "Target TCP pose not achieved."
            e_txt2 = " Expected: Pose {},".format(arr_1[:3])
            e_txt3 = " Recieved: Pose {}.".format(arr_2[:3])

        else:
            e_txt1 = "Target Joint pose not achieved."
            e_txt2 = " Expected: Pose {},".format(arr_1[:3])
            e_txt3 = " Recieved: Pose {}.".format(arr_2[:3])
        e_txt = e_txt1 + e_txt2 + e_txt3
        assert_wrapper(ret, e_txt)

    return ret


def move_joints(jnt_0, jnt_f, tcp_f, speed_lim_jnt, speed_lim_tcp,
                rob_frequency, dist, tcp_accuracy, jnt_accuracy):
    """
    Moves the robot to target joint positions, by breaking down the delta into
    smaller deltas to sync with the frequency of robot communication.

    """
    global last_target

    assert jnt_f is not None
    assert jnt_0 is not None
    if last_target is not None:
        jnt_0 = last_target.copy()

    jnt_diff = jnt_f - jnt_0
    vel = jnt_diff * rob_frequency
    max_vel_ovrshoot = max(vel - speed_lim_jnt)
    delta_jnt = max(ceil(max_vel_ovrshoot), 1)

    delta = max(dist * rob_frequency/speed_lim_tcp, delta_jnt)
    jnt_diff = jnt_f - jnt_0

    jnt_delta = jnt_diff/delta
    jnt_ = jnt_0.copy()
    t = None
    for i in range(int(ceil(delta))):
        jnt_ = jnt_ + jnt_delta
        THREAD.rob.send_joint_positions(jnt_, rob_frequency, THREAD.step)
        j, t = THREAD.manual_step()
        time.sleep(1/rob_frequency)

    while not (check_if_equal(jnt_f, THREAD.state.joint_positions, jnt_accuracy) and
               check_if_equal(tcp_f, t, tcp_accuracy, pose=True)):
        THREAD.rob.send_joint_positions(jnt_f, rob_frequency, THREAD.step)
        j, t = THREAD.manual_step()

    last_target = np.array(jnt_f.copy())

    check_if_equal(THREAD.state.joint_positions, jnt_f, jnt_accuracy,
                   assrt=True)


def move_robot(action, **kwargs):
    """
    Computes target joint values from actions and sends it to robot
    """

    jnt_acc = kwargs.get("jnt_accuracy")
    tcp_acc = kwargs.get("tcp_accuracy")
    jnt_0, tcp_0 = THREAD.manual_step()
    jnt_0_1, tcp_0_1 = rt.get_modified_joints(THREAD.rob, tcp_0,
                                              jnt_0, trans=action)

    move_joints(jnt_0, jnt_0_1, tcp_0_1, **kwargs)

    jnt_1, tcp_1 = THREAD.manual_step()

    check_if_equal(jnt_1, jnt_0_1, jnt_acc, assrt=True)
    check_if_equal(tcp_1, tcp_0_1, tcp_acc, pose=True, assrt=True)

    return jnt_0, jnt_0_1, tcp_1


class RobotCommunication(threading.Thread):
    """
    Connection thread to continuously fetch the robot state
    """
    def __init__(self, robot_interface, frequency):
        assert_wrapper(robot_interface is not None,
                       "Invalid robot interface")
        self.rob = robot_interface
        self._frequency = frequency
        self.state = None
        self.running = True
        self.step = 0
        threading.Thread.__init__(self)

    def run(self):
        global thread_stopped
        try:
            thread_stopped = False

            while self.running:
                self.state = self.get_state()
        except Exception as e:
            thread_stopped = True
            raise_wrapper(e)

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
                time.sleep(1/self._frequency)
                self.rob.connect()
        return state

    def manual_step(self):
        jnt = self.state.joint_positions
        tcp = rt.extract_tcp(self.rob.forward_kinematics(jnt))
        self.step += 1
        return jnt, tcp

    def close(self):
        """
        Shutdowns Thread and close the connection to the robot
        """
        self.running = False
        self.join()

        # Release Control
        self.rob.release_control()
        self.rob.disconnect()
        print("{} Disconnected".format(self.rob.get_model()))


def gen_random_actions(dim=3, dist=0.1):
    """
    Generate action sets in random order.
    Args:
        dim: Number of axes to move in.
        dist: Length of action in m
    """
    import random
    actions = []
    ax = [2, 0, 1]
    for i in range(dim):
        action = [0, 0, 0]
        action[ax[i]] = dist
        actions.insert(len(actions), action.copy())
        action[ax[i]] = -dist
        actions.insert(len(actions), action)
    random.shuffle(actions)
    return actions


def extract_path(path):
    """
    Extract path from string
    """

    if path.startswith("./"):
        path = os.getcwd() + path[1:]

    from pathlib import Path

    if not Path(path).is_file():
        print("FileNotFoundError: Robot implementation not found at path"
              ": {}".format(path))
        exit()

    return path


def parse_args():
    parser = ArgumentParser(description="Micropsi Industries Robot SDK Tool")
    parser._action_groups.pop()
    required = parser.add_argument_group("requried arguments")
    optional = parser.add_argument_group("optional arguments")

    required.add_argument("-p", "--path", required=True, metavar='\b',
                          help="path to the Robot implementation")
    required.add_argument("-r", "--robot", required=True, metavar='\b',
                          help="Robot Model as defined in the implementation")
    optional.add_argument("-f", "--frequency", default=DEF_FREQUENCY,
                          metavar='\b', type=float,
                          help="Frequency of the Robot in Hz. Default: {}Hz."
                          " Maximum Frequency {}"
                          "".format(DEF_FREQUENCY, MAX_FREQUENCY))
    optional.add_argument("-sp", "--tcp_speed", default=DEFAULT_TCP_SPEED,
                          type=float, metavar='\b', help=" TCP "
                          "speed in meter per second Default: {}, "
                          "Max: {}".format(DEFAULT_TCP_SPEED, MAX_TCP_SPEED))
    optional.add_argument("-d", "--dimension", default="3", type=int,
                          metavar='\b', help="Number of Axis to move "
                          "the robot in. Default: 3")

    optional.add_argument("-l", "--length", default=LENGTH, type=float,
                          metavar='\b', help="Length of movement in meters, "
                          "Default:{}m, Max: {}m".format(LENGTH, MAX_LENGTH))

    optional.add_argument("-ip", "--ip", default=DEFAULT_IP, metavar='\b',
                          help="IP address of the robot. Default:"
                          " {}".format(DEFAULT_IP))

    optional.add_argument("-j", "--joint_accuracy", default=DEFAULT_ACC,
                          type=float, metavar='\b', help="Accuracy of the "
                          "robot joints. Default: {}".format(DEFAULT_ACC))
    optional.add_argument("-t", "--tcp_accuracy", default=DEFAULT_ACC,
                          type=float, metavar='\b',
                          help="Accuracy of the TCP position achieved by "
                          "robot. Default: {}".format(DEFAULT_ACC)),
    optional.add_argument("-db", "--debug", action="store_true",
                          help="(Flag) Enable traceback of failed tests.")
    return parser.parse_args()


def main():
    global THREAD, DEBUG
    args = parse_args()
    path = args.path
    robot_model = args.robot
    robot_ip = args.ip
    DEBUG = args.debug

    robot_frequency = args.frequency if args.frequency <= MAX_FREQUENCY else MAX_FREQUENCY
    dimensions = args.dimension if (args.dimension < 4 and args.dimension > 0) else 1
    dist = args.length if args.length <= MAX_LINEAR_MOVEMENT else MAX_LINEAR_MOVEMENT
    jnt_accuracy = args.joint_accuracy if args.joint_accuracy <= ACCURACY_MAX else ACCURACY_MAX
    tcp_accuracy = args.tcp_accuracy if args.tcp_accuracy <= ACCURACY_MAX else ACCURACY_MAX
    tcp_speed_lim = args.tcp_speed if args.tcp_speed <= 0.1 else 0.1

    path = extract_path(path)

    collection = RobotInterfaceCollection()
    robot_path = os.path.expanduser(path)
    collection.load_interface_file(robot_path)
    supported_robots = sorted(collection.list_robots())

    if len(supported_robots) == 0:
        raise_wrapper(NotImplementedError,
                      err_txt=" No Robot Implementation found")

    try:
        supported_robots.index(robot_model)
    except ValueError as e:
        raise_wrapper(e, err_txt="Unknown/unsupported Robot model")

    robot_interface = collection.get_robot_interface(robot_model)

    robot_kwargs = {
        "frequency": robot_frequency,
        "model": robot_model,
        "ip_address": robot_ip,
    }

    guide_with_internal_sensor = False
    if robot_interface.has_internal_ft_sensor():
        guide_with_internal_sensor = True
        robot_kwargs["guide_with_internal_sensor"] = guide_with_internal_sensor
    rob = robot_interface(**robot_kwargs)

    if rob is None:
        raise_wrapper()

    THREAD = RobotCommunication(rob, robot_frequency)
    assert_wrapper(rob.get_model() is robot_model,
                   "Invalid Robot model loaded")

    print("Robot {} implementation found and loaded".format(rob.get_model()))
    try:
        print("Attempting to connect to Robot")
        assert_wrapper(rob.connect(), "Robot connection")
        THREAD.start()
        while THREAD.state is None and not thread_stopped:
            time.sleep(0.1)
        print("Connected to Robot {}".format(robot_model))
        jnt_speed_lim = rob.get_joint_speed_limits()
        assert_wrapper(THREAD.state is not None, "Invalid Robot State")

        jnt_cnt = rob.get_joint_count()
        jnt_speed_lmt = rob.get_joint_speed_limits()
        jnt_pos_lmt = rob.get_joint_position_limits()
        has_internal_ft = rob.has_internal_ft_sensor()

        if has_internal_ft and guide_with_internal_sensor:
            assert_wrapper(len(THREAD.state.raw_wrench) == 6,
                           "Invalid FT data")

        jnt_e = "Invalid joint positions"
        assert_wrapper(len(THREAD.state.joint_positions) == jnt_cnt, jnt_e)
        assert_wrapper(len(jnt_pos_lmt) == jnt_cnt, jnt_e)
        assert_wrapper(len(jnt_speed_lmt) == jnt_cnt, jnt_e)

        rob.connect()

        rob.release_control()
        rob.prepare_for_control()

        jnt_0, tcp_0 = THREAD.manual_step()

        rob.release_control()
        rob.prepare_for_control()
        rob.take_control()

        kwargs = {
            "rob_frequency": robot_frequency,
            "speed_lim_jnt": jnt_speed_lim,
            "speed_lim_tcp": tcp_speed_lim,
            "jnt_accuracy": jnt_accuracy,
            "tcp_accuracy": tcp_accuracy,
            "dist": dist
        }
        print("Moving in {} axes, with distance {}".format(dimensions, dist))

        # Send move to current position instruction to protect against
        # outdated move instructions in Robot register.
        move_joints(jnt_0, jnt_0, tcp_0, **kwargs)
        check_if_equal(jnt_0, THREAD.state.joint_positions, jnt_accuracy,
                       assrt=True)

        actions = gen_random_actions(dimensions, dist=dist)

        for i in range(len(actions)):
            print("Moving to: Position  {}".format(i + 1))
            j0, jf, tf = move_robot(actions[i], **kwargs)
            time.sleep(2.1)

        check_if_equal(tcp_0, tf, tcp_accuracy, pose=True, assrt=True)
        check_if_equal(jnt_0, jf, jnt_accuracy, assrt=True)

    except Exception as e:
        try:
            THREAD.close()
        except:
            pass

        if thread_stopped:
            raise_wrapper(None, err_txt="Robot communication failed",
                          error_title="ERROR")
        raise e

    THREAD.close()


if __name__ == '__main__':
    main()
