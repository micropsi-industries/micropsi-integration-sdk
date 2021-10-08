import os

import numpy as np


def invert_transform(*, matrix: np.ndarray):
    new_rotate = np.transpose(matrix[:3, :3])
    new_translate = np.dot(-new_rotate, matrix[:3, 3])
    new_matrix = np.identity(4)
    new_matrix[0:3, 0:3] = new_rotate
    new_matrix[0:3, 3] = new_translate
    return new_matrix


def gen_random_movements(dim, dist):
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
