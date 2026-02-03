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
    Generate translation matrices for motion tests.

    Args:
        max_distance: maximum translation distance in meters

    Returns:
        actions: list of 4x4 transformation matrices
        descriptions: a string for each action, for logging

    Generates translations: +x, -x, +y, -y, +z, -z
    """
    actions = []
    descriptions = []
    axis_names = ['x', 'y', 'z']

    for axis in range(3):
        for sign in [1, -1]:
            m = np.identity(4)
            m[axis, 3] = sign * max_distance
            actions.append(m)
            descriptions.append(f"{'+' if sign > 0 else '-'}{axis_names[axis]} ({max_distance:.3f}m)")

    return actions, descriptions


def generate_single_rotations(max_distance_degrees):
    """
    Generate rotation matrices for motion tests.

    Args:
        max_distance_degrees: maximum rotation angle in degrees

    Returns:
        actions: list of 4x4 transformation matrices
        descriptions: a string for each action, for logging

    Generates:
    - Single-axis rotations: +rx, -rx, +ry, -ry, +rz, -rz
    - Diagonal axis rotations (xy, yz, xz)
    """
    actions = []
    descriptions = []
    axis_names = ['x', 'y', 'z']
    rdist = np.deg2rad(max_distance_degrees)

    # Single-axis rotations: +rx, -rx, +ry, -ry, +rz, -rz
    axis_vectors = [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]
    for axis_idx, axis_vec in enumerate(axis_vectors):
        for sign in [1, -1]:
            actions.append(_rotation_matrix(axis_vec, sign * rdist))
            descriptions.append(f"rot {'+' if sign > 0 else '-'}{axis_names[axis_idx]} ({max_distance_degrees:.1f}°)")

    # Rotations around 45° diagonal axes, to test combined rotations
    diagonal_axes = [
        (np.array([1, 1, 0]), "xy"),
        (np.array([0, 1, 1]), "yz"),
        (np.array([1, 0, 1]), "xz"),
    ]
    for axis_vec, axis_name in diagonal_axes:
        axis_normalized = axis_vec / np.linalg.norm(axis_vec)
        for sign in [1, -1]:
            actions.append(_rotation_matrix(axis_normalized, sign * rdist))
            descriptions.append(f"rot {'+' if sign > 0 else '-'}{axis_name}-diagonal ({max_distance_degrees:.1f}°)")

    return actions, descriptions


def generate_chained_rotations(max_distance_degrees):
    """
    Generate a Z-Y-X rotation sequence (and its inverse) for motion tests.

    Args:
        max_distance_degrees: rotation angle in degrees

    Returns:
        actions: list of 4x4 transformation matrices
        descriptions: a string for each action, for logging

    Generates consecutive rotations: +rz, +ry, +rx, -rx, -ry, -rz
    """
    actions = []
    descriptions = []
    angle_rad = np.deg2rad(max_distance_degrees)

    sequence = [
        (np.array([0, 0, 1]), +angle_rad, "sequence: +rz"),
        (np.array([0, 1, 0]), +angle_rad, "sequence: +ry"),
        (np.array([1, 0, 0]), +angle_rad, "sequence: +rx"),
        (np.array([1, 0, 0]), -angle_rad, "sequence: -rx"),
        (np.array([0, 1, 0]), -angle_rad, "sequence: -ry"),
        (np.array([0, 0, 1]), -angle_rad, "sequence: -rz"),
    ]
    for axis, angle, label in sequence:
        actions.append(_rotation_matrix(axis, angle))
        descriptions.append(f"{label} ({max_distance_degrees:.1f}°)")

    return actions, descriptions


def _rotation_matrix(axis, angle_rad):
    """
    Create a 4x4 rotation matrix around an arbitrary axis.

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
