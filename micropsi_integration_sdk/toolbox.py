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


def generate_actions(do_translation, do_rotation, max_distance_translation, max_distance_degrees):
    """
    Generate a series of transformation matrices for motion tests.

    Returns:
        actions: list of 4x4 transformation matrices
        descriptions: a string for each action, for logging

    - If translation enabled: translate +/- along x, y, z axes
    - If rotation enabled: rotate +/- around x, y, z axes
    - If rotation enabled: diagonal axis rotations (xy, yz, xz)
    """
    actions = []
    descriptions = []
    axis_names = ['x', 'y', 'z']
    dist = max_distance_translation
    rdist = np.deg2rad(max_distance_degrees)

    if do_translation:
        # Translations: +x, -x, +y, -y, +z, -z
        for axis in range(3):
            for sign in [1, -1]:
                m = np.identity(4)
                m[axis, 3] = sign * dist
                actions.append(m)
                descriptions.append(f"{'+' if sign > 0 else '-'}{axis_names[axis]} ({dist:.3f}m)")

    if do_rotation:
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
