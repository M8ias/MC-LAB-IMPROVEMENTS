#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float64MultiArray
from messages.msg import observer_message, reference_message
import dynamic_reconfigure.client

class controller():
    def __init__(self):
        self.x = self.square = self.circle = self.triangle = self.rightArrow = self.leftArrow = self.upArrow = self.DownArrow = self.L1 = self.R1 = self.L2 = self.R2 = self.L3 = self.R3 = self.share = self.options = self.PS = self.pad = 0
        self.lStickX = self.lStickY = self.rStickX = self.rStickY = self.L2A = self.R2A = 0.0

    def updateState(self, data):
        self.x = data.buttons[3]
        self.square = data.buttons[0]
        self.circle = data.buttons[2]
        self.triangle = data.buttons[1]
        self.rightArrow = data.buttons[16]
        self.leftArrow = data.buttons[14]
        self.upArrow = data.buttons[15]
        self.DownArrow = data.buttons[17]
        self.L1 = data.buttons[4]
        self.R1 = data.buttons[6]
        self.L2 = data.buttons[5]
        self.R2 = data.buttons[7]
        self.L3 = data.buttons[12]
        self.R3 = data.buttons[13]
        self.options = data.buttons[9]
        self.share = data.buttons[8]
        self.PS = data.buttons[10]
        self.pad = data.buttons[11]

        self.lStickX = -data.axes[0]
        self.lStickY = data.axes[1]
        self.rStickX = -data.axes[2]
        self.rStickY = data.axes[3]
        self.L2A = data.axes[4]
        self.R2A = data.axes[5]
    
class UVector():
    def __init__(self):
        #self.leftRotorThrust = 0.0
        #self.rightRotorThrust = 0.0
        #self.bowRotorThrust = 0.0
        #self.leftRotorAngle = 0.0
        #self.rightRotorAngle = 0.0
        self.Udata = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub = 0
        self.message = Float64MultiArray()

    def publish(self, data):
        self.message.data = data
        self.pub.publish(self.message)
        

# Stuff to publish the computed tau
class Tau():
    def __init__(self):
        self.tau = np.array([0, 0, 0])

    def updateTau(self, msg):
        self.tau = msg.data
    
    def getTau(self):
        return self.tau


class Reference_Conversationalist():
    """
    The Reference_Conversationalist object deals with publishing and subscribing 
    to the CSEI/reference topic. New values are stored in this object when the topic is updated.
    """
    def __init__(self):
        self.reference_msg = reference_message()
        self.pub = rospy.Publisher('CSEI/reference/', reference_message, queue_size=1)
        self.eta_d = np.array([0, 0, 0])
        self.nu_d = np.array([0, 0, 0])
        self.nu_d_dot = np.array([0, 0 ,0])

    def callback_reference_data(self, msg):
        self.eta_d = msg.eta_d
        self.nu_d = msg.nu_d
        self.nu_d_dot = msg.nu_d_dot
    
    def publish_reference_data(self, eta_d, nu_d, nu_d_dot):
        self.reference_msg.eta_d = eta_d
        self.reference_msg.nu_d = nu_d
        self.reference_msg.nu_d_dot = nud_d_dot
        self.pub.publish(self.reference_msg)
       
    def get_observer_data(self):
        return self.eta_d, self.nu_d, self.nu_d_dot
    

class Observer_Listener:
    """ 
    The Observer_Listener object listens to the /CSEI/observer topic. It is 
    """
    def __init__(self):
        self.observer_msg = observer_message()
        self.pub = rospy.Publisher('CSEI/observer/', observer_message, queue_size=1)
        self.eta_hat = np.array([0, 0, 0])
        self.nu_hat = np.array([0, 0, 0])
        self.bias_hat = np.array([0, 0 ,0])

    def callback_observer_data(self, msg):
        self.eta_hat = msg.eta
        self.nu_hat = msg.nu
        self.bias_hat = msg.bias

    def get_observer_data(self):
        return self.eta_hat, self.nu_hat, self.bias_hat

class Controller_Gains():
    def __init__(self):
        self.Kp = np.zeros(3)
        self.Kd = np.zeros(3)
        self.Ki = np.zeros(3)
        self.mu = 0
        self.Uref = 0

    def get_controller_gains(self):
        return self.Kp, self.Kd, self.Ki, self.mu, self.Uref

    def callback(self, config):
        self.Kp = self.string2array(config.Kp)
        self.Kd = self.string2array(config.Kd)
        self.Ki = self.string2array(config.Ki)
        self.mu = config.mu
        self.Uref = config.U_ref

    def string2array(self, string):
        return np.array(list(map(int, string.split(','))))

ps4 = controller()
Udata = UVector()
tau = Tau()
reference = Reference_Conversationalist()
observer = Observer_Listener()
Gains = Controller_Gains()



def controllNodeInit():
    global node
    node = rospy.init_node('stud_controll_node')
    rospy.Subscriber("joy", Joy, ps4.updateState)
    rospy.Subscriber("CSEI/observer", observer_message, observer.callback_observer_data)
    rospy.Subscriber("CSEI/reference", reference_message, reference.callback_reference_data)
    gain_client = dynamic_reconfigure.client.Client('parameter_server_basics', timeout=30, config_callback = Gains.callback)
    Udata.pub = rospy.Publisher('CSEI/u', Float64MultiArray, queue_size = 1)

def nodeEnd():
    global node
    node.destroy_node()

