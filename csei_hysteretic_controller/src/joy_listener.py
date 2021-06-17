import rospy
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float64MultiArray


global pub

def updateJoy(data):
    global ps4
    ps4.updateState(data.axes, data.buttons)

def updateObserverOdometry(data):
    global observerOdometry
    observerOdometry = data

def updateU(data):
    global Udata
    Udata = data 

Udata = Float64MultiArray()
observerOdometry = Odometry()

class controller():
    def __init__(self):
        self.x = self.square = self.circle = self.triangle = self.rightArrow = self.leftArrow = self.upArrow = self.DownArrow = self.L1 = self.R1 = self.L2 = self.R2 = self.L3 = self.R3 = self.share = self.options = self.PS = self.pad = 0
        self.lStickX = self.lStickY = self.rStickX = self.rStickY = self.L2A = self.R2A = 0.0

    def updateState(self, axes, buttons):
        self.x = buttons[3]
        self.square = buttons[0]
        self.circle = buttons[2]
        self.triangle = buttons[1]
        self.rightArrow = buttons[16]
        self.leftArrow = buttons[14]
        self.upArrow = buttons[15]
        self.DownArrow = buttons[17]
        self.L1 = buttons[4]
        self.R1 = buttons[6]
        self.L2 = buttons[5]
        self.R2 = buttons[7]
        self.L3 = buttons[12]
        self.R3 = buttons[13]
        self.options = buttons[9]
        self.share = buttons[8]
        self.PS = buttons[10]
        self.pad = buttons[11]

        self.lStickX = axes[0]
        self.lStickY = axes[1]
        self.rStickX = axes[2]
        self.rStickY = axes[3]
        self.L2A = axes[4]
        self.R2A = axes[5]



class U():
    def __init__(self):
        self.leftRotorThrust = 0.0
        self.rightRotorThrust = 0.0
        self.bowRotorThrust = 0.0
        self.leftRotorAngle = 0.0
        self.rightRotorAngle = 0.0
    

ps4 = controller()

def getControllerState():
    global ps4
    return ps4

def getObserverOdometry():
    global observerOdometry
    return observerOdometry

def getU():
    return Udata

def controllNodeInit():
    global pub, node
    node = rospy.init_node('stud_controll_node')
    rospy.Subscriber("joy", Joy, updateJoy)
    rospy.Subscriber("CSEI/observer/odom", Odometry, updateObserverOdometry)
    pub = rospy.Publisher('CSEI/u', Float64MultiArray, queue_size=1)

def nodeEnd():
    node.destroy_node()

message = Float64MultiArray()

def publishU(data):
    global pub
    message.data = [data.leftRotorThrust, data.rightRotorThrust, data.bowRotorThrust, data.leftRotorAngle, data.rightRotorAngle]
    pub.publish(message)
