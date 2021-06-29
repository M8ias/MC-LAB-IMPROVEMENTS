import rospy
import numpy as np
import math
from lib import qualisys, Tau, Observer


### Student code ###

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

### End of student code ###

def loop():
    
    tau = Tau.getTau
    eta = qualisys.getQualisysOdometry()
    old_eta_hat, old_nu_hat, old_bias_hat = Observer.get_observer_data()
    eta_hat, nu_hat, bias_hat = linear_observer(old_eta_hat, old_nu_hat, old_bias_hat, eta, tau)
    Observer.publish_observer_data(eta_hat, nu_hat, bias_hat)
    return 0