import numpy as np
import math


def Rzyx(psi):
    """
    Rzyx(psi) computes the rotation matrix, R in SO(3), using the
    zyx convention and Euler angle representation.
    """

    R = np.array([[math.cos(psi), -math.sin(psi), 0],
                  [math.sin(psi), math.cos(psi), 0],
                  [0, 0, 1]])
    return R
