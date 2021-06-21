import rospy
import numpy as np
import math
from lib import qualisys, Udata
### Student code ###
def compute_tau(u):
        u_t = np.transpose(np.take(u, [0, 1, 2])[np.newaxis])
        alpha = np.take(u, [3, 4])
        c1 = math.cos(alpha[0])
        c2 = math.cos(alpha[1])
        c3 = 0
        s1 = math.sin(alpha[0])
        s2 = math.sin(alpha[1])
        s3 = 1
        L = np.array([[-0.4574, -0.4574, 0.3875], [-0.055, 0.055, 0]]) 
        K = np.diag(np.diag([1.03, 1.03, 2.629]))
        B = np.array([[c1, c2, c3], [s1, s2, s3], [L[0][0]*s1 - L[1][0]*c1, L[0][1]*s1 - L[1][1]*c1, L[0][2]*s3 - L[1][2]*c3]])
        tau = (K @ B) @ u_t)
        return tau

def linear_ship_dynamics(eta_hat, nu_hat, bias_hat, eta, tau, dt):
    """
    Observer
    """
    M = np.array([[16.11, 0.0, 0.0], [0.0, 24.11, 0.5291], [0.0, 0.5291, 2.7600]])
    D = np.array([[0.66, 0., 0.], [0., 1.3, 2.8], [0., 0., 1.9]])
    L_1 = np.diag([10.0, 10.0, 10.0])
    L_2 = np.diag([50.0, 50.0, 50.0])
    L_3 = np.diag([1.0, 1.0, 1.0])
    R = get_rotation_matrix(eta[2])
    M_inv = np.linalg.inv(M)

    eta_tilde = eta - eta_hat
    eta_hat_dot = R @ nu_hat + L_1 @ eta_tilde

    nu_hat_dot = M_inv @ (-D @ nu_hat + R.T @
                        bias_hat + tau + R.T @ L_2 @ eta_tilde)
    bias_hat_dot = L_3 @ eta_tilde

    eta_hat = eta_hat + dt * eta_hat_dot
    nu_hat  = nu_hat + dt * nu_hat_dot
    bias_hat = bias_hat + dt * bias_hat_dot

    return eta_hat, nu_hat, bias_hat

### End of student code ###

def loop():
    u = Udata.getU
    tau = compute_tau(u)
    dt = 0.01 

    eta 
    old_eta_hat
    old_nu_hat
    old_bias_hat

    eta_hat, nu_hat, bias_hat = linear_ship_dynamics(old_eta_hat, old_nu_hat, old_bias_hat, eta, tau, dt, )
    
    print(Udata.getU())

    return 0