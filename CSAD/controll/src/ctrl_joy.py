import rospy
import math
import numpy as np
form lib import ps4, Udata
form std_msgs import Float64MultiArray

def saturate(u):
    if u < -0.5:
        u = -0.5
    if u > 0.5:
        u = 0.5
    return u

def sixaxis2thruster(lStickX, lStickY, rStickX, rStickY):
    """
    sixaxis2thruster controlls enables the user to control the from actuators with the left ps4 stick, 
    and the back thrusters with the right thrusters
    """
    u = np.zeros(12)

    ### Front actuators ###

    u[0:2] = saturate(math.sqrt(lStickX ** 2 + lStickY ** 2)) # Input
    u[6:8] = math.atan2(lStickX, -lStickY) + math.pi # Angle

    ### Back actuators ###

    u[3:5] = saturate(math.sqrt(rStickX**2 + rStickY**2))
    u[9:11] = math.atan2(rStickX, -rStickY)

    return u

    def loop():
        u = sixaxis2thruster(ps4.lStickX, ps4.lStickY, ps4.rStickX, ps4.rStickY)