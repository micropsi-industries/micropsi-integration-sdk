"""
Helper for sandbox.py, generating incremental pose changes towards a waypoint.

This is bascially identical to MIRAI's go_to_goal node ('rubberband effect') .
"""

import numpy as np
from numpy.linalg import norm
from pyquaternion import Quaternion


def get_step_towards_goal(
    current_xyz, current_q, goal_xyz, goal_q, stepsize_xyz, stepsize_rot, slowdown_steps
):
    """Get a stepsize-limited translation & rotation from some pose to another.

    Args:
        current_xyz: numpy.ndarray 3-vector containing the current
            cartesian position of the TCP in m
        current_q: pyquaternion.Quaternion containing the current pose
            quaternion of the TCP
        goal_xyz: numpy.ndarray 3-vector containing the target cartesian
            TCP position
        goal_q: pyquaternion.Quaternion containing the target TCP pose
            quaternion
        stepsize_xyz: float
            Maximum TCP translational step size in m
        stepsize_rot: float
            Maximum TCP rotational step size in degrees
        slowdown_steps: scalar
            Step lengths are scaled down such at any given time, we would need
            at least `slowdown_steps` steps from now on to reach the goal.
            This has the effect of asymptotically slowing us down near the goal,
            a simple way to avoid overshoots.

    Returns:
        action_rot: pyqaternion.Quaternion the rotation motion
    """

    assert isinstance(current_q, Quaternion)
    assert isinstance(goal_q, Quaternion)

    dist_xyz = norm(goal_xyz - current_xyz)
    dist_rot = abs((goal_q * current_q.inverse).degrees)

    stepsize_xyz_m, stepsize_rot_m = _equalise_stepsizes(dist_xyz,
                                                         dist_rot,
                                                         stepsize_xyz,
                                                         stepsize_rot)

    # asymptocically decrease the stepsize to always stay
    # N steps away from the goal
    if slowdown_steps * stepsize_xyz_m > dist_xyz:
        stepsize_xyz_m = dist_xyz / slowdown_steps

    if slowdown_steps * stepsize_rot_m > dist_rot:
        stepsize_rot_m = dist_rot / slowdown_steps

    action_xyz = _generate_xyz_action(current_xyz, goal_xyz, stepsize_xyz_m)
    action_rot = _generate_rot_action(current_q, goal_q, stepsize_rot_m)

    return action_xyz, action_rot


def _equalise_stepsizes(dist_xyz, dist_rot, stepsize_xyz, stepsize_rot):
    """Reduces the stepsize_xyz or the stepsize_rot such that the distance to the cartesian
    and rotational target (dist_xyz and dist_rot) can be achieved in the same total
    number steps.

    Args:
        dist_xyz (float): the cartesian distance in meters that needs to
            be covered
        dist_rot (float): the rotational distance in degrees to be
            covered
        stepsize_xyz (float): The allowed translational step size of the
            TCP in m
        stepsize_rot (float): The allowed rotational step size of the
            TCP in degrees

    Returns:
        tuple: tuple (stepsize_xyz, stepsize_rot), the new equalised
        stepsizes.
    """
    expected_xyz_steps = dist_xyz / stepsize_xyz
    expected_rot_steps = dist_rot / stepsize_rot

    if expected_xyz_steps > expected_rot_steps:
        stepsize_rot = dist_rot / float(expected_xyz_steps)

    elif expected_xyz_steps < expected_rot_steps:
        stepsize_xyz = dist_xyz / float(expected_rot_steps)

    return stepsize_xyz, stepsize_rot


def _generate_xyz_action(current_xyz, goal_xyz, stepsize):
    """Generate a length-limited cartesian displacement from one point to another.

    This draws a direct line from the current pose to the goal pose,
    but goes at most `stepsize` meters for any one time this is called.

    Args:
        current_xyz (numpy.ndarray): 3-vector contain the current xyz-
            position of the TCP
        goal_xyz (numpy.ndarray): 3-vector contain the target xyz-
            position being aimed at
        stepsize (float): maximum allowed magnitude of the displacement

    Returns:
        3-vector numpy.ndarray
    """
    if np.allclose(current_xyz, goal_xyz):
        return np.zeros(3)

    difference = goal_xyz - current_xyz
    distance = norm(difference)
    direction = difference / distance
    action = difference if (distance < stepsize) else (stepsize * direction)

    return action


def _generate_rot_action(current_q, goal_q, stepsize):
    """Generate a distance-limited rotation from one pose to another.

    This interpolates between two quaternions using Slerp, going at most
    `stepsize` degrees along the way from one to the other (less if they
    are already closer)

    Args:
        current_q (pyquaternion.Quaternion): contain the current
            quaternion pose of the TCP
        goal_q (pyquaternion.Quaternion): 4-vector contain the target
            pose
        stepsize (float): max rotational stepsize in degrees

    Returns:
        pyquaternion.Quaternion: Returned pose
    """
    assert isinstance(current_q, Quaternion)
    assert isinstance(goal_q, Quaternion)

    theta = (goal_q * current_q.inverse).degrees

    if abs(theta) < 10 ** -10:
        return Quaternion()

    # Most interpolation methods are sensitive to the same rotation
    # represented as q or -q
    if theta < 0:
        goal_q = -goal_q

    # Limit to the fraction of the motion that's achievable in this step
    alpha = stepsize / abs(theta)
    alpha = 1 if alpha >= 1 else alpha
    q_int = Quaternion.slerp(current_q, goal_q, alpha)
    action = q_int * current_q.inverse

    return action

