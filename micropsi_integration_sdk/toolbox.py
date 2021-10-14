import os

import numpy as np


def invert_transform(*, matrix: np.ndarray):
    new_rotate = np.transpose(matrix[:3, :3])
    new_translate = np.dot(-new_rotate, matrix[:3, 3])
    new_matrix = np.identity(4)
    new_matrix[0:3, 0:3] = new_rotate
    new_matrix[0:3, 3] = new_translate
    return new_matrix


def generate_actions(*, dimensions: int, distance: float):
    """
    Generate action sets.
    Args:
        dimensions: Number of axes to move in.
        distance: Length of action in m
    """
    actions = []
    for i in range(dimensions):
        action = [0, 0, 0]
        action[i] = distance
        actions.append(action.copy())
        action[i] = -distance
        actions.append(action.copy())
    return actions


def extract_path(path):
    """
    Extract path from string
    """
    path = os.path.expanduser(path)
    path = os.path.abspath(path)
    return path
