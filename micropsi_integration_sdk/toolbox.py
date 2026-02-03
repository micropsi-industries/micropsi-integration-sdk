import os

import numpy as np
from scipy.spatial.transform import Rotation


def invert_transform(*, matrix: np.ndarray):
    new_rotate = np.transpose(matrix[:3, :3])
    new_translate = np.dot(-new_rotate, matrix[:3, 3])
    new_matrix = np.identity(4)
    new_matrix[0:3, 0:3] = new_rotate
    new_matrix[0:3, 3] = new_translate
    return new_matrix


def generate_translations(max_distance):
    """
    Generate a sequence of translations: +x, -x, +y, -y, +z, -z.

    After this sequence, the robot should be back to where it started.

    Args:
        max_distance: maximum translation distance in meters

    Returns:
        actions: list of 4x4 transformation matrices
        descriptions: a string for each action, for logging
    """
    actions = []
    descriptions = []
    axis_names = ['X', 'Y', 'Z']

    for axis in range(3):
        for sign in [1, -1]:
            m = np.identity(4)
            m[axis, 3] = sign * max_distance
            actions.append(m)
            descriptions.append(f"linear {'+' if sign > 0 else '-'}{axis_names[axis]} ({int(1000*max_distance)}mm)")

    return actions, descriptions


def generate_single_rotations(max_distance_degrees):
    """
    Generate a sequence of rotations:

    - Single-axis rotations: +rx, -rx, +ry, -ry, +rz, -rz
    - Diagonal axis rotations: +xy, -xy, +yz, -yz, +xz, -xz

    After this sequence, the robot should be back to where it started.

    Args:
        max_distance_degrees: maximum rotation angle in degrees

    Returns:
        actions: list of 4x4 transformation matrices
        descriptions: a string for each action, for logging
    """
    actions = []
    descriptions = []
    axis_names = ['X', 'Y', 'Z']
    rdist = np.deg2rad(max_distance_degrees)

    # Single-axis rotations: +rx, -rx, +ry, -ry, +rz, -rz
    axis_vectors = [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]
    for axis_idx, axis_vec in enumerate(axis_vectors):
        for sign in [1, -1]:
            actions.append(_rotation_to_transformation_matrix(axis_vec, sign * rdist))
            descriptions.append(f"rotation {'+' if sign > 0 else '-'}{axis_names[axis_idx]} ({max_distance_degrees:.1f}°)")

    # Rotations around 45° diagonal axes, to test combined rotations
    diagonal_axes = [
        (np.array([1, 1, 0]), "XY"),
        (np.array([0, 1, 1]), "YZ"),
        (np.array([1, 0, 1]), "XZ"),
    ]
    for axis_vec, axis_name in diagonal_axes:
        axis_normalized = axis_vec / np.linalg.norm(axis_vec)
        for sign in [1, -1]:
            actions.append(_rotation_to_transformation_matrix(axis_normalized, sign * rdist))
            descriptions.append(f"rotation {'+' if sign > 0 else '-'}{axis_name}-diagonal ({max_distance_degrees:.1f}°)")

    return actions, descriptions


def generate_chained_rotations(max_distance_degrees):
    """
    Generate a Z-Y-X rotation sequence, followed by the reverse

    Args:
        max_distance_degrees: rotation angle in degrees

    Returns:
        actions: list of 4x4 transformation matrices
        descriptions: a string for each action, for logging
    """
    actions = []
    descriptions = []
    angle_rad = np.deg2rad(max_distance_degrees)

    sequence = [
        (np.array([0, 0, 1]), +angle_rad, "+Z"),
        (np.array([0, 1, 0]), +angle_rad, "+Y"),
        (np.array([1, 0, 0]), +angle_rad, "+X"),
        (np.array([1, 0, 0]), -angle_rad, "-X"),
        (np.array([0, 1, 0]), -angle_rad, "-Y"),
        (np.array([0, 0, 1]), -angle_rad, "-Z"),
    ]
    for axis, angle, label in sequence:
        actions.append(_rotation_to_transformation_matrix(axis, angle))
        descriptions.append(f"rotation {label} ({max_distance_degrees:.1f}°)")

    return actions, descriptions


def _rotation_to_transformation_matrix(axis, angle_rad):
    """
    Create a 4x4 transformation matrix representing a pure rotation

    Args:
        axis: unit vector [x, y, z] defining the rotation axis
        angle_rad: rotation angle in radians
    """
    assert np.allclose(np.linalg.norm(axis), 1.0)
    rot = Rotation.from_rotvec(axis * angle_rad)
    m = np.identity(4)
    m[:3, :3] = rot.as_matrix()
    return m



def extract_path(path):
    """
    Extract path from string
    """
    path = os.path.expanduser(path)
    path = os.path.abspath(path)
    return path
