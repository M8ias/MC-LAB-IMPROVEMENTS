import rospy
from sensor_msgs.msg import Joy
import numpy as np
from std_msgs.msg import Float64MultiArray



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

        self.lStickX = -data.axes[1]
        self.lStickY = data.axes[0]
        self.rStickX = -data.axes[3]
        self.rStickY = data.axes[2]
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

    

def updateObserverOdometry(data):
    global odometry
    odometry = data



ps4 = controller()
Udata = UVector()

### Node tools ###

def controllNodeInit():
    global node
    node = rospy.init_node('stud_controll_node')
    rospy.Subscriber("joy", Joy, ps4.updateState)
    Udata.pub = rospy.Publisher('CSEI/u', Float64MultiArray, queue_size = 1)

def nodeEnd():
    global node
    node.destroy_node()

