from numpy.lib.function_base import _DIMENSION_NAME
from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection
from operator import pos
from argparse import ArgumentParser

import numpy as np
import os
import math
import time
import threading

DEFAULT_IP = "192.168.100.100"
MAX_TCP_SPEED = 0.1
DEFAULT_TCP_SPEED = 0.1
ACCURACY_MAX = 0.1
JNT_ACCURACY = 1e-2
TCP_ACCURACY = 1e-2

MAX_LINEAR_MOVEMENT = 0.1
MAX_FREQUENCY = 50
FREQUENCY = 20

LENGTH = 0.05
MAX_LENGTH = 0.1


LAST_TARGET = None
STATE = None
RUNNING = True
MOVING = False
MOVE_REQ = False
THREAD = None


def get_state(rob):
    state = None
    while state is None:
        state = rob.get_hardware_state()
        if state is None:
            rob.clear_cached_hardware_state()
            time.sleep(1)
            rob.connect()
    assert state is not None
    return state


def set_R_with_fixed_XYZ(new_T, orientation):
    from math import sin, cos
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
    new_T = np.identity(4)
    new_T[0:3, 3] = position
    new_T = set_R_with_fixed_XYZ(new_T, orientation)
    return new_T


def get_modified_joints(rob, tcp, jnt, trans=[0., 0., 0.], rot=[0., 0., 0.]):

    tcp_2 = np.zeros(6)
    for i in range(3):
        tcp_2[i] = tcp[i] + trans[i]
        tcp_2[3+i] = tcp[3+i] + rot[i]

    tf = create_with_fixed_angle_pose(tcp_2[:3], tcp_2[3:])
    jnt_2 = rob.inverse_kinematics(tf, jnt)
    return jnt_2, tcp_2


def check_if_equal(arr_1, arr_2, E, pose=False):

    arr1 = arr_1.copy()
    arr2 = arr_2.copy()
    assert len(arr1) == len(arr2)
    res = True
    if pose:
        arr1[3:6] = [0., 0., 0]
        arr2[3:6] = [0., 0., 0]

    for i in range(len(arr1)):
        delta = arr1[i] - arr2[i]
        if not abs(delta) < E:
            return False
    return True


def get_nearest_equivalent(a1, a0=0):
    return (a1 - (a0 - np.pi)) % (2 * np.pi) + (a0 - np.pi)


def signed_angle_resolution(T, sign=+1, reference=(0, 0, 0)):
    from math import atan2
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
    candidates = []

    for sign in [-1, +1]:

        solution = signed_angle_resolution(tcp_0, sign, reference)
        candidates.append(solution)

    loss0 = np.sum(np.abs(candidates[0] - reference))
    loss1 = np.sum(np.abs(candidates[1] - reference))

    if loss0 < loss1:
        return candidates[0]
    else:
        return candidates[1]


def get_orientation_as_fixed_XYZ(tcp_0, reference=(0, 0, 0)):
    rx, ry, rz = get_major_angles(tcp_0, reference)
    return [rx, ry, rz]


def fix_tcp(tcp_0):

    t = np.zeros(6)
    t[:3] = tcp_0[0:3, 3]
    t[3:] = get_orientation_as_fixed_XYZ(tcp_0)
    return t


def manual_step(rob, step):
    jnt = STATE.joint_positions
    tcp = fix_tcp(rob.forward_kinematics(jnt))
    return jnt, tcp, step+1


def move_joints(rob, jnt_f, jnt_0, step, SPEED_LIM_JNT,SPEED_LIM_TCP, rob_frequency, dist):
    global LAST_TARGET, MOVING, MOVE_REQ
    import time

    assert jnt_f is not None
    assert jnt_0 is not None
    if LAST_TARGET is not None:
        jnt_0 = LAST_TARGET.copy()

    jnt_diff = jnt_f - jnt_0
    vel = jnt_diff * rob_frequency
    max_vel_ovrshoot = max(vel - SPEED_LIM_JNT)
    delta_jnt = max(math.ceil(max_vel_ovrshoot), 1)

    sec = dist / SPEED_LIM_TCP
    delta = max(dist * rob_frequency/SPEED_LIM_TCP, delta_jnt)
    jnt_diff = jnt_f - jnt_0

    jnt_delta = jnt_diff/delta
    jnt_ = jnt_0.copy()

    for i in range (int(math.ceil(delta))):
        jnt_ = jnt_ + jnt_delta
        MOVE_REQ = True
        rob.send_joint_positions(jnt_, rob_frequency, step)
        j, t, step = manual_step(rob, step)
        time.sleep(1/rob_frequency)

    while not check_if_equal(jnt_f, STATE.joint_positions, JNT_ACCURACY):
        rob.send_joint_positions(jnt_f, rob_frequency, step)
        j, t, step = manual_step(rob, step)

    MOVE_REQ = False
    LAST_TARGET = np.array(jnt_f.copy())
    while MOVING or MOVE_REQ:
        time.sleep(0.001)

    assert check_if_equal(STATE.joint_positions, jnt_f, JNT_ACCURACY)

    return step


def move_robot(rob, step, action,dist, **kwargs):
    jnt_0, tcp_0, step = manual_step(rob, step)
    jnt_0_z, tcp_0_z = get_modified_joints(rob, tcp_0, jnt_0, trans=action)

    step = move_joints(rob, jnt_0_z, jnt_0, step,dist=dist, **kwargs)

    jnt_1, tcp_1, step = manual_step(rob, step)

    assert check_if_equal(jnt_1, jnt_0_z, JNT_ACCURACY)
    assert check_if_equal(tcp_1, tcp_0_z, TCP_ACCURACY, pose=True)

    return step, jnt_0, jnt_0_z, tcp_1


def close(THREAD, rob):
    global RUNNING

    # Close Thread
    if THREAD is not None and THREAD.is_alive():
        RUNNING = False
        THREAD.join()
    # Release Control
    if rob is not None:
        rob.release_control()
        rob.disconnect()
    print("{} Disconnected".format(rob.get_model()))


def read(robot_interface, frequency):
    global STATE, RUNNING, MOVING, MOVE_REQ

    assert robot_interface is not None
    cnt = 0
    last_posj = None

    time.sleep(1/frequency)
    while (RUNNING):
        t1 = time.time()
        STATE = get_state(robot_interface)
        if last_posj is not None:
            if check_if_equal(last_posj, STATE.joint_positions, 0.0001):
                MOVING = False
            else:
                MOVING = True
                MOVE_REQ = False
        last_posj = STATE.joint_positions.copy()


def gen_random_actions(dim=3, dist=0.1):
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


def get_path(path):
    if path.startswith("./"):
        import os
        path = os.getcwd() + path[1:]

    from pathlib import Path

    if not Path(path).is_file():
        raise FileNotFoundError(path)

    return path


def parse_args():

    parser = ArgumentParser()
    parser.add_argument("-p", "--path", required=True,
                        help="path to the Robot implementation")
    parser.add_argument("-r", "--robot_model", required=True, 
                        help="Robot Model as defined in the implementation")
    parser.add_argument("-f", "--frequency", default=FREQUENCY,
                        help="Frequency of the Robot in Hz. Default: {}Hz."
                        " Maximum Frequency {}".format(FREQUENCY, MAX_FREQUENCY))
    parser.add_argument("-sp", "--tcp_speed", default=DEFAULT_TCP_SPEED, help=" TCP "
                        "speed in meter per second Default: {}, "
                        "Max: {}".format(DEFAULT_TCP_SPEED,MAX_TCP_SPEED))
    parser.add_argument("-d", "--dimension", default="3",
                        help="Number of Axis to move the robot in. Default: 3")

    parser.add_argument("-l", "--length", default=LENGTH, help="Length of movement in "
                        "meters, Default:{} meters, Max: {}"
                        "".format(LENGTH, MAX_LENGTH))

    parser.add_argument("-ip", "--ip_address", default=DEFAULT_IP, help="IP address of "
                        "the robot. Default: {}".format(DEFAULT_IP))

    parser.add_argument("-j", "--joint_accuracy", default=JNT_ACCURACY,  help="Accuracy "
                        "of the robot joints Default: {}".format(JNT_ACCURACY))
    parser.add_argument("-t", "--tcp_accuracy", default=TCP_ACCURACY, help="Accuracy of "
                        "the TCP position achieved by robot. "
                        "Default: {}".format(TCP_ACCURACY))

    return parser.parse_args()


def main():
    global THREAD
    args = parse_args()

    path = args.p
    robot_model = args.r
    robot_frequency = float(args.f) if float(args.f) <= MAX_FREQUENCY else MAX_FREQUENCY
    dimensions = int(args.d) if (int(args.d) < 4 and int(args.d) > 0) else 1
    dist = float(args.l) if float(args.l) <= MAX_LINEAR_MOVEMENT else MAX_LINEAR_MOVEMENT
    robot_ip = args.ip
    JNT_ACCURACY = float(args.aj) if float(args.aj) <= ACCURACY_MAX else ACCURACY_MAX
    TCP_ACCURACY = float(args.at) if float(args.at) <= ACCURACY_MAX else ACCURACY_MAX
    TCP_SPEED_LIM = float(args.sp) if float(args.sp) <= 0.1 else 0.1

    path = get_path(path)

    print("Moving in {} axes, with distance {}".format(dimensions, dist))

    collection = RobotInterfaceCollection()

    robot_path = os.path.expanduser(path)
    collection.load_interface_file(robot_path)
    supported_robots = sorted(collection.list_robots())

    if len(supported_robots) == 0:
        raise NotImplementedError(" No Robot Implementation found")

    if robot_model is None and len(supported_robots) == 1:
        print("Robot model not specified, Starting robot "
              "{}".format(supported_robots[0]))
        robot_model = supported_robots[0]
    else:
        try:
            supported_robots.index(robot_model)
        except ValueError:
            raise NotImplementedError("Unknown/unsupported Robot model")

    robot_interface = collection.get_robot_interface(robot_model)

    print(supported_robots)
    print(robot_model)

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

    THREAD = threading.Thread(target=read, args=(rob, robot_frequency))
    assert rob.get_model() is robot_model
    try:
        assert rob.connect()
        THREAD.start()
        print("Connected to Robot {}".format(robot_model))
        JOINT_SPEED_LIM = rob.get_joint_speed_limits()
        time.sleep(0.1)
        assert STATE is not None
        jnt_cnt = rob.get_joint_count()
        jnt_speed_lmt = rob.get_joint_speed_limits()
        jnt_pos_lmt = rob.get_joint_position_limits()
        has_internal_ft = rob.has_internal_ft_sensor()

        while STATE is None:
            time.sleep(0.1)

        if has_internal_ft and guide_with_internal_sensor:
            assert len(STATE.raw_wrench) == 6

        assert len(STATE.joint_positions) == jnt_cnt
        assert len(jnt_pos_lmt) == jnt_cnt
        assert len(jnt_speed_lmt) == jnt_cnt
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
                "SPEED_LIM_JNT": JOINT_SPEED_LIM,
                "SPEED_LIM_TCP": TCP_SPEED_LIM
        }

        move_joints(rob, jnt_0, jnt_0, step,dist=dist, **kwargs)
        actions = gen_random_actions(dimensions, dist=dist)

        for i in range(len(actions)):
            print("Moving to: Position  {}".format(i + 1))
            step, j0, jf, tf = move_robot(rob, step, actions[i],dist=dist, **kwargs)
            time.sleep(2.1)

        assert check_if_equal(tcp_0, tf, TCP_ACCURACY, pose=True)
        assert check_if_equal(jnt_0, jf, JNT_ACCURACY)

    except Exception as e:
        close(THREAD, rob)
        err = ""
        if type(e) is AssertionError:
            err = "Assertion Failed"
            raise Exception(err)

        raise e

    close(THREAD, rob)


if __name__ == '__main__':
    main()
