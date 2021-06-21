import rospy
from lib import odometry, ps4, Udata
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

"""
stud_code_joy_controller.py is to contain all thrust allocation algorithms. 
"""

# all buttons = 1 while pressed and 0 while not pressed
# ps4.x -- for x button
# ps4.square -- for square button
# ps4.circle -- for circle button
# ps4.triangle -- for triangle button
# ps4.rightArrow -- for right arrow button
# ps4.leftArrow  -- for left arrow button 
# ps4.upArrow -- for up arrow button
# ps4.DownArrow  -- for down arrow button
# ps4.L1 -- for left top trigger
# ps4.R1 -- for right top trigger
# ps4.L2 -- for left bottom trigger
# ps4.R2 -- for right bottom trigger
# ps4.L3 -- for left joystick press
# ps4.R3 -- for right joystick press
# ps4.options -- for options button
# ps4.share -- for share button
# ps4.PS -- for ps button
# ps4.pad -- for pad pressed

# stick values are mapped from -1.0 to 1.0, trigger values are mapped from 0.0 to 1.0
# ps4.lStickX -- left stick right to left value, right is negative left is positive
# ps4.lStickY -- left stick up down value, up is positive down is negative
# ps4.rStickX -- right stick right to left value, right is negative left is positive
# ps4.rStickY -- right stick up down value, up is positive down is negative
# ps4.L2A -- left trigger analog value
# ps4.R2A -- right trigger analog value


# odometry.pose.pose.orientation.x
# odometry.pose.pose.orientation.y
# odometry.pose.pose.orientation.z
# odometry.pose.pose.orientation.w
# odometry.pose.pose.position.x
# odometry.pose.pose.position.y
# odometry.pose.pose.position.z
# odometry.twist.twist.angular.x
# odometry.twist.twist.angular.y
# odometry.twist.twist.angular.z
# odometry.twist.twist.linear.x
# odometry.twist.twist.linear.y
# odometry.twist.twist.linear.z

# Udata.publish(data) where data is a 5 elements long vector containing five floats, publishes your controll vector to ROS.
# [
# leftRotorThrust,
# rightRotorThrust,
# bowRotorThrust,
# leftRotorAngle,
# rightRotorAngle
# ]

# Deafault and should always be here
def sixaxis2thruster(lStickX, lStickY, rStickX, rStickY, R2, L2):
    """
    sixaxis2thruster()directly maps the sixaxis playstation controller inputs
    to the vessel actuators.
    """
    ### Acutator commands ###
    u1 = (R2 - L2)
    u2 = math.sqrt(lStickX ** 2  + lStickY ** 2)
    u3 = math.sqrt(rStickX ** 2 + rStickY ** 2)


    ### VSD angles ###
    alpha1 = math.atan2(lStickY, lStickX)
    alpha2 = math.atan2(rStickY, rStickX)

    u = np.array([u1, u2, u3, alpha1, alpha2])
    return u


def input_mapping(lStickX, lStickY, rStickX, rStickY, R2, L2):
    """
    This function maps the input from the joystick to corresponding deegre of
    freedom. Inputs are from controller, and returns a generelized force 
    vector tau. 
    """
    surge = (lStickY + rStickY)
    sway = (lStickX + rStickX)
    yaw = 0.5*(R2 + L2)

    tau = np.array([[surge], [sway], [yaw]])
    return tau


def extended_thrust_allocation(tau):
    """ 
    An extended thrust algorithm
    """ 
    # Positional value for the thrusters [m]
    lx = np.array([-0.4574, -0.4574, 0.3875])
    ly = np.array([-0.055, 0.055, 0])
    u = np.zeros(5)

    B_ext = np.array([[0, 1, 0, 1, 0], [1, 0, 1, 0, 1], [lx[2], -ly[0], lx[0], -ly[1], lx[1]]])
    K =np.array([
        [2.629, 0, 0, 0, 0],
        [0, 1.030, 0, 0, 0],
        [0, 0, 1.030, 0, 0],
        [0, 0, 0, 1.030, 0],
        [0, 0, 0, 0, 1.030]
    ])

    inv_matrix = np.linalg.pinv(np.dot(B_ext, K))
    u_ext = inv_matrix @ tau
    u[0] = u_ext[0]
    u[1] = math.sqrt(u_ext[1]**2 + u_ext[2]**2)
    u[2] = math.sqrt(u_ext[3]**2 + u_ext[4]**2)
    u[3] = math.atan2(u_ext[2], u_ext[1])
    u[4] = math.atan2(u_ext[4], u_ext[3])
    return u

def loop():
    # Call your thrust allocation algorithm here. 
    u = sixaxis2thruster(ps4.lStickX, ps4.lStickY, ps4.rStickX, ps4.rStickY, ps4.R2, ps4.L2)
    Udata.publish(u)
