from math import tau
from Kinematics import quat2eul
import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from messages.msg import observer_message
import numpy as np

class qualisys():
    def __init__(self):
        self.odom = Odometry()
        self.eta = np.zeros(3)

    def updateQualisysOdometry(self, data):
        self.odom = data

    def getQualisysOdometry(self):
        w = self.odom.pose.pose.orientation.w
        x = self.odom.pose.pose.orientation.x
        y = self.odom.pose.pose.orientation.y
        z = self.odom.pose.pose.orientation.z

        self.eta[0] = self.odom.pose.pose.position.x
        self.eta[1] = self.odom.pose.pose.position.y
        self.eta[2] = quat2eul(w, x, y, z)
        
        return self.eta


class UVector():
    #leftRotorThrust = 0.0
    #rightRotorThrust = 0.0
    #bowRotorThrust = 0.0
    #leftRotorAngle = 0.0
    #rightRotorAngle = 0.0
    Udata = [0.0, 0.0, 0.0, 0.0, 0.0]

    def updateU(self, message):
        self.Udata = message.data

    def getU(self):
        return self.Udata 


class Tau():
    def __init__(self):
        self.tau = np.array([0, 0, 0])

    def updateTau(self, msg):
        self.tau = msg.data
    
    def getTau(self):
        return self.tau

class Observer():
    def __init__(self):
        self.observer_msg = observer_message()
        self.pub = rospy.Publisher('CSEI/observer/', observer_message, queue_size=1)
        self.eta_hat = np.array([0, 0, 0])
        self.nu_hat = np.array([0, 0, 0])
        self.bias_hat = np.array([0, 0 ,0])

    def callback_observer(self, msg):
        self.eta_hat = msg.eta
        self.nu_hat = msg.nu
        self.bias_hat = msg.bias
    
    def publish_observer_data(self, eta_hat, nu_hat, bias_hat):
        self.observer_msg.eta = eta_hat
        self.observer_msg.nu = nu_hat
        self.observer_msg.bias = bias_hat
        self.pub.publish(self.observer_msg)

    def get_observer_data(self):
        return self.eta_hat, self.nu_hat, self.bias_hat

qualisys = qualisys()
Udata = UVector()
Observer = Observer()
Tau  = Tau()


def observerNodeInit():
    global pub, node
    node = rospy.init_node('stud_odom_node')
    rospy.Subscriber("/qualisys/CSEI/odom", Odometry, qualisys.updateQualisysOdometry)
    rospy.Subscriber("CSEI/u", Float64MultiArray, Udata.updateU)
    rospy.Subscriber("CSEI/observer", observer_message, Observer.callback_observer)
    rospy.Subscriber("CSEI/tau", Float64MultiArray, Tau.updateTau)
   

def nodeEnd():
    node.destroy_node()