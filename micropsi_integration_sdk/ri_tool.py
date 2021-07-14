from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection
from operator import pos
from argparse import ArgumentParser
from numpy import rad2deg as dd

import numpy as np
import os
import math
import time
import threading

JNT_ACCURACY = 1e-6
TCP_ACCURACY = 1e-4

last_target = None
state = None
running = True
moving = False
move_requested = False
thread = None


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
    jnt = state.joint_positions
    tcp = fix_tcp(rob.forward_kinematics(jnt))
    return jnt, tcp, step+1


def move_joints(rob, jnt_f, jnt_0, step, SPEED_LIM, rob_frequency):
    global last_target, moving, move_requested

    assert jnt_f is not None
    assert jnt_0 is not None
    if last_target is not None:
        jnt_0 = last_target.copy()
    jnt_diff = jnt_f - jnt_0
    vel = jnt_diff * rob_frequency

    max_vel_ovrshoot = max(vel - SPEED_LIM)
    factor = max(math.ceil(max_vel_ovrshoot),1)# * 2

    jnt_ = jnt_0.copy()
    jnt_delta = jnt_diff/factor

    for i in range(factor):
        jnt_ = jnt_ + jnt_delta
        move_requested = True
        while not check_if_equal(jnt_, state.joint_positions, JNT_ACCURACY):
            rob.send_joint_positions(jnt_, rob_frequency, step)
            j, t, step = manual_step(rob, step)

        if not check_if_equal(jnt_0, jnt_f, JNT_ACCURACY):
            while move_requested and not moving:
                print("Waiting for robot to start moving")
                time.sleep(0.01)
        move_requested = False
    rob.send_joint_positions(jnt_f, rob_frequency, step)

    import time
    move_requested = False
    last_target = np.array(jnt_f.copy())
    while moving or move_requested:
        time.sleep(0.01)

    assert check_if_equal(state.joint_positions, jnt_f, JNT_ACCURACY)

    return step


def mover(rob, step, action, **kwargs):
    jnt_0, tcp_0, step = manual_step(rob, step)
    jnt_0_z, tcp_0_z = get_modified_joints(rob, tcp_0, jnt_0, trans=action)

    step = move_joints(rob, jnt_0_z, jnt_0, step, **kwargs)

    jnt_1, tcp_1, step = manual_step(rob, step)

    assert check_if_equal(jnt_1, jnt_0_z, JNT_ACCURACY)
    assert check_if_equal(tcp_1, tcp_0_z, TCP_ACCURACY, pose=True)

    return step, jnt_0, jnt_0_z, tcp_1


def close(thread, rob):
    global running

    # Close Thread
    if thread is not None and thread.is_alive():
        running = False
        thread.join()
    # Release Control
    rob.release_control()
    rob.disconnect()
    print("{} Disconnected".format(rob.get_model()))


def read(robot_interface, frequency):
    global state, running, moving, move_requested

    assert robot_interface is not None
    cnt = 0
    last_posj = None

    time.sleep(1/frequency)
    while (running):
        t1 = time.time()
        state = get_state(robot_interface)
        if last_posj is not None:
            if check_if_equal(last_posj, state.joint_positions, 0.0001):
                moving = False
            else:
                moving = True
                move_requested = False

        last_posj = state.joint_positions.copy()


def gen_random_actions(dim=3, dist=0.1):
    import random
    actions = []
    ax = [1, 0, 2]
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

    #for Testing
    res = [i for i, ltr in enumerate(path) if ltr == "/"]
    fpath = path[:res[len(res)-1]+1]

    return path, fpath


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("--r", "--robot_model", required=True)
    parser.add_argument("--p", "--path", required=True)
    parser.add_argument("--d", "--directions")
    return parser.parse_args()


def main():
    global thread
    args = parse_args()
    robot_model = args.r
    dimensions = args.d
    path = args.p

    path, fpath = get_path(path)
    if dimensions is None or int(dimensions) > 4 or int(dimensions) < 0:
        dimensions = 3
    else:
        dimensions = int(dimensions)
    print("Moving in {} axes".format(dimensions))

    collection = RobotInterfaceCollection()

    import sys
    sys.path.append(fpath)
    robot_path = os.path.expanduser(path)
    collection.load_interface_file(robot_path)
    supported_robots = sorted(collection.list_robots())

    if len(supported_robots) == 0:
        raise NotImplementedError(" No Robot Implementation found")

    if robot_model is None and len(supported_robots)==1:
        print("Robot model not specified, Starting robot {}"
              "".format(supported_robots[0]))
        robot_model = supported_robots[0]
    else:
        try:
            supported_robots.index(robot_model)
        except ValueError:
            raise NotImplementedError("Unknown/unsupported Robot model")

    robot_interface = collection.get_robot_interface(robot_model)

    print(supported_robots)
    print(robot_model)

    robot_ip = "192.168.100.100"
    robot_frequency = 20
    guide_with_internal_sensor = False
    robot_kwargs = {
            "frequency": robot_frequency,
            "model": robot_model,
            "ip_address": robot_ip,
    }

    if robot_interface.has_internal_ft_sensor():
        guide_with_internal_sensor = True
        robot_kwargs["guide_with_internal_sensor"] = guide_with_internal_sensor
    rob = robot_interface(**robot_kwargs)

    thread = threading.Thread(target=read, args=(rob, robot_frequency))
    assert rob.get_model() is robot_model
    assert rob.connect()
    thread.start()
    print("Connected to Robot {}".format(robot_model))
    JOINT_SPEED_LIM = rob.get_joint_speed_limits()
    time.sleep(1)
    assert state is not None
    try:

        jnt_cnt = rob.get_joint_count()
        jnt_speed_lmt = rob.get_joint_speed_limits()
        jnt_pos_lmt = rob.get_joint_position_limits()
        has_internal_ft = rob.has_internal_ft_sensor()

        while state is None:
            time.sleep(0.1)

        if has_internal_ft and guide_with_internal_sensor:
            assert len(state.raw_wrench) == 6

        assert len(state.joint_positions) == jnt_cnt
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
                "SPEED_LIM": JOINT_SPEED_LIM
        }

        move_joints(rob, jnt_0, jnt_0, step, **kwargs)
        actions = gen_random_actions(dimensions, dist=0.1)

        for i in range(len(actions)):
            print("Moving: Position  {}".format(i + 1))
            step, j0, jf, tf = mover(rob, step, actions[i], **kwargs)
            time.sleep(2.1)

        assert check_if_equal(tcp_0, tf, TCP_ACCURACY, pose=True)
        assert check_if_equal(jnt_0, jf, JNT_ACCURACY)

    except Exception as e:
        close(thread, rob)
        err = ""
        if type(e) is AssertionError:
            err = "Assertion Failed"
            raise Exception(err)

        raise e

    close(thread, rob)


if __name__ == '__main__':
    main()
