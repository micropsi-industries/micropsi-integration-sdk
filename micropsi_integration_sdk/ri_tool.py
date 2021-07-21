from argparse import ArgumentParser

import os
import time
import threading
from math import sin, cos, atan2, ceil
import numpy as np
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

LAST_TARGET = None
STATE = None
RUNNING = True
MOVING = False
MOVE_REQ = False
THREAD = None
thread_stopped = True


def set_R_with_fixed_XYZ(new_T, orientation):
    rx = orientation[0]
    ry = orientation[1]
    rz = orientation[2]
    # Fixed Angle xyz

    r11 = cos(rz) * cos(ry)
    r21 = sin(rz) * cos(ry)
    r31 = -sin(ry)

    r12 = cos(rz) * sin(ry) * sin(rx) - sin(rz) * cos(rx)
    r22 = sin(rz) * sin(ry) * sin(rx) + cos(rz) * cos(rx)
    r32 = cos(ry) * sin(rx)

    r13 = cos(rz) * sin(ry) * cos(rx) + sin(rz) * sin(rx)
    r23 = sin(rz) * sin(ry) * cos(rx) - cos(rz) * sin(rx)
    r33 = cos(ry) * cos(rx)

    new_R = np.asarray([
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]])

    new_T[0:3, 0:3] = new_R
    return new_T


def create_with_fixed_angle_pose(position, orientation):
    """ Creates TF matrix from position and orientation"""
    new_T = np.identity(4)
    new_T[0:3, 3] = position
    new_T = set_R_with_fixed_XYZ(new_T, orientation)
    return new_T


def get_modified_joints(rob, tcp, jnt, trans=[0., 0., 0.], rot=[0., 0., 0.]):
    """
    Gets joint target from the actions.
    """

    tcp_2 = np.zeros(6)
    for i in range(3):
        tcp_2[i] = tcp[i] + trans[i]
        tcp_2[3+i] = tcp[3+i] + rot[i]

    tf = create_with_fixed_angle_pose(tcp_2[:3], tcp_2[3:])
    jnt_2 = rob.inverse_kinematics(tf, jnt)
    return jnt_2, tcp_2


def check_if_equal(arr_1, arr_2, E, pose=False,
                   assrt=False, rob=None):
    """
    Checks if two incomming poses are equivalent (delta < E).
    For pose only the linear component of position is verified
    """
    ret = True
    arr1 = arr_1.copy()
    arr2 = arr_2.copy()

    p = "TCP" if pose else "Joint"
    assert_wrapper(len(arr1) == len(arr2), "Invalid {} pose".format(p), rob)

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
        assert_wrapper(ret, e_txt, rob)

    return ret


def get_nearest_equivalent(a1, a0=0):
    return (a1 - (a0 - np.pi)) % (2 * np.pi) + (a0 - np.pi)


def signed_angle_resolution(T, sign=+1, reference=(0, 0, 0)):
    """Represent a rot. matrix as one of two possible triples of angles."""
    assert sign == +1 or sign == -1

    cosay_squared = T[0, 0]**2 + T[1, 0]**2
    cosay = sign * np.sqrt(cosay_squared)
    sinay = -T[2, 0]

    if cosay != 0:

        ay_representative = atan2(sinay, cosay)
        ay = get_nearest_equivalent(ay_representative, reference[1])

        cosaz = T[0, 0] / cosay
        sinaz = T[1, 0] / cosay

        az_representative = atan2(sinaz, cosaz)
        az = get_nearest_equivalent(az_representative, reference[2])

        cosax = T[2, 2] / cosay
        sinax = T[2, 1] / cosay

        ax_representative = atan2(sinax, cosax)
        ax = get_nearest_equivalent(ax_representative, reference[0])

    else:

        sin_ax_minus_az = T[0, 1]
        cos_ax_minus_az = T[1, 1]
        ax_minus_az = atan2(sin_ax_minus_az, cos_ax_minus_az)

        ay = get_nearest_equivalent(sign * np.pi / 2, reference[1])
        az = reference[2]
        ax = ax_minus_az + az

        assert ax == reference[0]

    return np.array([ax, ay, az])


def get_major_angles(tcp_0, reference=(0, 0, 0)):
    """
    Represent a rotation matrix as three angles of major-axis rotation.
    """
    candidates = []

    for sign in [-1, +1]:

        solution = signed_angle_resolution(tcp_0, sign, reference)
        candidates.append(solution)

    loss0 = np.sum(np.abs(candidates[0] - reference))
    loss1 = np.sum(np.abs(candidates[1] - reference))

    if loss0 < loss1:
        return candidates[0]
    return candidates[1]


def get_orientation_as_fixed_XYZ(tcp_0, reference=(0, 0, 0)):
    rx, ry, rz = get_major_angles(tcp_0, reference)
    return [rx, ry, rz]


def extract_tcp(tcp_0):
    """
    Converts TF matrix into 6d array with fixed angle orientation
    """
    t = np.zeros(6)
    t[:3] = tcp_0[0:3, 3]
    t[3:] = get_orientation_as_fixed_XYZ(tcp_0)
    return t


def manual_step(rob, step):
    jnt = STATE.joint_positions
    tcp = extract_tcp(rob.forward_kinematics(jnt))
    return jnt, tcp, step+1


def move_joints(rob, jnt_0, jnt_f, tcp_f, step, speed_lim_jnt, speed_lim_tcp,
                rob_frequency, dist, tcp_accuracy, jnt_accuracy):
    """
    Moves the robot to target joint positions, by breaking down the delta into
    smaller deltas to sync with the frequency of robot communication.

    """
    global LAST_TARGET, MOVING, MOVE_REQ

    assert jnt_f is not None
    assert jnt_0 is not None
    if LAST_TARGET is not None:
        jnt_0 = LAST_TARGET.copy()

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
        MOVE_REQ = True
        rob.send_joint_positions(jnt_, rob_frequency, step)
        j, t, step = manual_step(rob, step)
        time.sleep(1/rob_frequency)

    while not (check_if_equal(jnt_f, STATE.joint_positions, jnt_accuracy) and
               check_if_equal(tcp_f, t, tcp_accuracy, pose=True)):
        rob.send_joint_positions(jnt_f, rob_frequency, step)
        j, t, step = manual_step(rob, step)

    MOVE_REQ = False
    LAST_TARGET = np.array(jnt_f.copy())
    while MOVING or MOVE_REQ:
        time.sleep(0.001)

    check_if_equal(STATE.joint_positions, jnt_f, jnt_accuracy,
                   assrt=True, rob=rob)

    return step


def move_robot(rob, step, action, **kwargs):
    """
    Computes target joint values from actions and sends it to robot
    """

    jnt_acc = kwargs.get("jnt_accuracy")
    tcp_acc = kwargs.get("tcp_accuracy")
    jnt_0, tcp_0, step = manual_step(rob, step)
    jnt_0_1, tcp_0_1 = get_modified_joints(rob, tcp_0, jnt_0, trans=action)

    step = move_joints(rob, jnt_0, jnt_0_1, tcp_0_1, step, **kwargs)

    jnt_1, tcp_1, step = manual_step(rob, step)

    check_if_equal(jnt_1, jnt_0_1, jnt_acc, assrt=True, rob=rob)
    check_if_equal(tcp_1, tcp_0_1, tcp_acc, pose=True, assrt=True, rob=rob)

    return step, jnt_0, jnt_0_1, tcp_1


def close(thread, rob):
    """
    Shutdowns Thread and close the connection to the robot
    """
    global RUNNING
    if thread is not None and thread.is_alive():
        RUNNING = False
        thread.join()

    # Release Control
    if rob is not None:
        rob.release_control()
        rob.disconnect()
    print("{} Disconnected".format(rob.get_model()))


class robot_communication(threading.Thread):
    """
    Connection thread to continuously fetch the robot state
    """
    def __init__(self, robot_interface, frequency):
        assert_wrapper(robot_interface is not None,
                       "Invalid robot interface")
        self._robot_interface = robot_interface
        self._frequency = frequency
        self._last_posj = None
        threading.Thread.__init__(self)

    def run(self):
        global STATE, RUNNING, MOVING, MOVE_REQ, thread_stopped
        try:
            thread_stopped = False

            while RUNNING:
                STATE = self.get_state()
                if self._last_posj is not None:
                    if check_if_equal(last_posj, STATE.joint_positions,
                                      DEFAULT_ACC):
                        MOVING = False
                    else:
                        MOVING = True
                        MOVE_REQ = False
                    last_posj = STATE.joint_positions.copy()
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
            state = self._robot_interface.get_hardware_state()
            if state is None:
                if cnt > 10:
                    raise InterruptedError("Invalid state recieved, check"
                                           " robot connection")

                self._robot_interface.clear_cached_hardware_state()
                time.sleep(0.5)
                self._robot_interface.connect()
        return state


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


def assert_wrapper(cond, txt, rob=None):
    """
    Assert wrapper, with text describing the test
    """
    global THREAD
    try:
        assert cond
    except Exception as e:
        try:
            close(THREAD, rob)
        except:
            pass
        raise_wrapper(e, st=txt)


def raise_wrapper(e, st="", st_0=""):
    if e is not None:
        if len(e.args) > 0:
            s = e.args[0]
        else:
            s = ""
        st_0 = type(e).__name__ + ": " + s

    print("{}. {}".format(st_0, st))

    exit()


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

    return parser.parse_args()


def main():
    global THREAD
    args = parse_args()
    path = args.path
    robot_model = args.robot
    robot_ip = args.ip


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
        raise_wrapper(NotImplementedError, st=" No Robot Implementation found")

    try:
        supported_robots.index(robot_model)
    except ValueError as e:
        raise_wrapper(e, st="Unknown/unsupported Robot model")

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

    THREAD = robot_communication(rob, robot_frequency)
    assert_wrapper(rob.get_model() is robot_model,
                   "Invalid Robot model loaded", rob)

    print("Robot {} implementation found and loaded".format(rob.get_model()))
    try:
        print("Attempting to connect to Robot")
        assert_wrapper(rob.connect(), "Robot connection", rob)
        THREAD.start()
        while STATE is None and not thread_stopped:
            time.sleep(0.1)
        print("Connected to Robot {}".format(robot_model))
        jnt_speed_lim = rob.get_joint_speed_limits()
        assert_wrapper(STATE is not None, "Invalid Robot State", rob)

        jnt_cnt = rob.get_joint_count()
        jnt_speed_lmt = rob.get_joint_speed_limits()
        jnt_pos_lmt = rob.get_joint_position_limits()
        has_internal_ft = rob.has_internal_ft_sensor()

        if has_internal_ft and guide_with_internal_sensor:
            assert_wrapper(len(STATE.raw_wrench) == 6, "Invalid FT data")

        jnt_e = "Invalid joint positions"
        assert_wrapper(len(STATE.joint_positions) == jnt_cnt, jnt_e, rob)
        assert_wrapper(len(jnt_pos_lmt) == jnt_cnt, jnt_e, rob)
        assert_wrapper(len(jnt_speed_lmt) == jnt_cnt, jnt_e, rob)

        rob.connect()

        rob.release_control()
        rob.prepare_for_control()

        step = 0
        jnt_0, tcp_0, step = manual_step(rob, step)

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
        move_joints(rob, jnt_0, jnt_0, tcp_0, step, **kwargs)
        actions = gen_random_actions(dimensions, dist=dist)

        for i in range(len(actions)):
            print("Moving to: Position  {}".format(i + 1))
            step, j0, jf, tf = move_robot(rob, step, actions[i], **kwargs)
            time.sleep(2.1)

        check_if_equal(tcp_0, tf, tcp_accuracy, pose=True, assrt=True, rob=rob)
        check_if_equal(jnt_0, jf, jnt_accuracy, assrt=True, rob=rob)

    except Exception as e:
        try:
            close(THREAD, rob)
        except:
            pass

        if thread_stopped:
            raise_wrapper(None, st="Robot communication failed", st_0="ERROR")
        raise e

    close(THREAD, rob)


if __name__ == '__main__':
    main()
