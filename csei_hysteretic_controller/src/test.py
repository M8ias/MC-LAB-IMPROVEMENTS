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
    return ucd


def linear_observer(eta_hat, nu_hat, bias_hat, eta, tau):
    """
    Observer
    """
    M = np.array([[16.11, 0.0, 0.0], [0.0, 24.11, 0.5291], [0.0, 0.5291, 2.7600]])
    D = np.array([[0.66, 0., 0.], [0., 1.3, 2.8], [0., 0., 1.9]])
    L_1 = np.diag([10.0, 10.0, 10.0])
    L_2 = np.diag([50.0, 50.0, 50.0])
    L_3 = np.diag([1.0, 1.0, 1.0])
    R = Rzyx(eta[2])
    M_inv = np.linalg.inv(M)
    dt = 0.01

    eta_tilde = eta - eta_hat
    eta_hat_dot = R @ nu_hat + L_1 @ eta_tilde

    nu_hat_dot = M_inv @ (-D @ nu_hat + R.T @
                        bias_hat + tau + R.T @ L_2 @ eta_tilde)
    bias_hat_dot = L_3 @ eta_tilde

    eta_hat = eta_hat + dt * eta_hat_dot
    nu_hat  = nu_hat + dt * nu_hat_dot
    bias_hat = bias_hat + dt * bias_hat_dot

    return eta_hat, nu_hat, bias_hat

eta_hat = np.array([1, 1, math.pi/4])
nu_hat = np.array([0.1, 0.1, 0.05])
bias_hat = np.array([0, 0, 0])
eta = np.eta([[1.1, 1.1, math.pi/4]])