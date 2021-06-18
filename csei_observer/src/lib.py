import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class qualisys():
    def __init__(self):
        self.observerOdometry = Odometry()

    def updateQualisysOdometry(self, data):
        self.observerOdometry = data

    def getQualisysOdometry(self):
        return (self.observerOdometry)

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

observer = qualisys()
Udata = UVector()

def observerNodeInit():
    global pub, node
    node = rospy.init_node('stud_odom_node')
    rospy.Subscriber("/qualisys/CSEI/odom", Odometry, observer.updateQualisysOdometry)
    rospy.Subscriber("CSEI/u", Float64MultiArray, Udata.updateU)
    pub = rospy.Publisher('CSEI/observer/odom', Odometry, queue_size=1)

def nodeEnd():
    node.destroy_node()