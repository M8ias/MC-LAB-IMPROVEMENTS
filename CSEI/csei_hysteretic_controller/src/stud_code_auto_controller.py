import rospy
import numpy as np
import math
from lib import odometry, ps4, Udata, Observer, Tau, Reference #Your message/publisher/subscriber stuff
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from Kinematics import Rzyx


def controller(eta_hat, eta_d, nu_hat, eta_prime, eta_2prime):
    psi = eta_hat[2]
    r = nu_hat[2]
    R = Rzyx(psi)
    S_psid = np.array([[0, -r, 0], [r, 0, 0], [0, 0, 0]])
    S_psi = np.array([[0, -psi, 0], [psi, 0, 0], [0, 0, 0]])

    z1 = R.T*(eta_hat - eta_d) #Error 
    # z1[2] = (z1[2] - math.pi) % (2*math.pi) - math.pi

    V1_prime = -z1 @ R.T @ eta_prime



def loop():
    Udata.publish([1.0, 1.1, 1.2, 1.3, 1.4])

    print(odometry.pose.pose.position.x)
    print(ps4.x)