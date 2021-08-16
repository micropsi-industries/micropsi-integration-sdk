import os
from math import sin, cos, atan2, sqrt

import numpy as np


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
    jnt_2 = rob.inverse_kinematics(end_effector_pose=tf, joint_reference=jnt)
    return jnt_2, tcp_2


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

def dist(p1,p2):
    return sqrt( ((p1[0]-p2[0])**2 ) + 
                 (  (p1[1]-p2[1])**2) +
                 (  (p1[2]-p2[2])**2) )


def gen_random_actions(dim, dist):
    """
    Generate action sets in random order.
    Args:
        dim: Number of axes to move in.
        dist: Length of action in m
    """
    import random
    actions = []
    ax = [0, 1, 2]
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
    path = os.path.expanduser(path)
    path = os.path.abspath(path)
    return path


def check_jnt_target(arr_1,arr_2, E):

    if not (len(arr_1) == 6 and len(arr_2) == 6):
        return False, "Invalid Joint poses"

    if not np.all(np.abs(arr_1 - arr_2) < E):
        ret_txt = ("""Target Joint pose not achieved.
        Expected: Pose {},
        Recieved: Pose {}.
        """.format(arr_1, arr_2))
        return False, ret_txt
    return True, ""


def check_tcp_target(arr_1,arr_2, E):

    if not (len(arr_1) == 6 and len(arr_2) == 6):
        return False, "Invalid TCP poses"

    if not np.all(np.abs(arr_1[:3] - arr_2[:3]) < E):
        ret_txt = ("""Target TCP pose not achieved.
        Expected: Pose x: {}, y: {}, z: {} 
        Recieved: Pose x: {}, y: {}, z: {}
        """.format(arr_1[0], arr_1[1], arr_1[2],
                   arr_2[0], arr_2[1], arr_2[2]))
        return False, ret_txt
    return True, ""
