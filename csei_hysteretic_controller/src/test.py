import numpy as np
import math

def test_allocation(tau):
    """ 
    An extended thrust algorithm
    """ 
    # Positional value for the thrusters [m]
    lx = np.array([-0.4574, -0.4574, 0.3875])
    ly = np.array([-0.055, 0.055, 0])
    u = np.zeros(5)

    B_ext = np.array([[1, 0, 1, 0, 0], [0, 1, 0, 1, 1], [-ly[0], lx[0], -ly[1], lx[1], lx[2]]])
    K =np.array([
        [1.030, 0, 0, 0, 0],
        [0, 1.030, 0, 0, 0],
        [0, 0, 1.030, 0, 0],
        [0, 0, 0, 1.030, 0],
        [0, 0, 0, 0, 2.629]
    ])

    inv_matrix = np.linalg.pinv(np.dot(B_ext, K))
    u_ext = np.dot(inv_matrix, tau)
    u[0] = math.sqrt(u_ext[0]**2 + u_ext[1]**2)
    u[1] = math.sqrt(u_ext[2]**2 + u_ext[3]**2)
    u[2] = u_ext[4]
    u[3] = math.atan2(u_ext[1], u_ext[0])
    u[4] = math.atan2(u_ext[3], u_ext[2])
    return u

tau  = np.array([1, 1, 0.5])
u = test_allocation(tau)

print(u)