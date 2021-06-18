from observer.observer.math_tools import quat2eul
import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

class qualisys():
    def __init__(self):
        self.qualisysOdometry = Odometry()

    def updateQualisysOdometry(self, data):
        self.qualisysOdometry = data

    def getQualisysOdometry(self):
        w = self.odom.pose.pose.orientation.w
        x = self.odom.pose.pose.orientation.x
        y = self.odom.pose.pose.orientation.y
        z = self.odom.pose.pose.orientation.z
        self = quat2eul(w, x, y, z)


        self.odom.pose.pose.position.x = self.eta[0]
        self.odom.pose.pose.position.y = self.eta[1]
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.w = quat[0]
        self.odom.pose.pose.orientation.x = quat[1]
        self.odom.pose.pose.orientation.y = quat[2]
        self.odom.pose.pose.orientation.z = quat[3]

        self.odom.twist.twist.linear.x = self.nu[0]
        self.odom.twist.twist.linear.y = self.nu[1]
        self.odom.twist.twist.linear.z = 0
        self.odom.twist.twist.angular.x = 0
        self.odom.twist.twist.angular.y = 0
        self.odom.twist.twist.angular.z = self.nu[2]


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

qualisys = qualisys()
Udata = UVector()

def observerNodeInit():
    global pub, node
    node = rospy.init_node('stud_odom_node')
    rospy.Subscriber("/qualisys/CSEI/odom", Odometry, qualisys.updateQualisysOdometry)
    rospy.Subscriber("CSEI/u", Float64MultiArray, Udata.updateU)
    rospy.Subscriber("CSEI/tau", Float64MultiArray, queue_size=1)
    rospy.Subscriber("CSEI/observer/eta", Vector3, queue_size=1)
    rospy.Subscriber("CSEI/observer/nu", Vector3, queue_size=1)
    rospy.Subscriber("CSEI/observer/bias", Vector3, queue_size=1)
    pub_eta = rospy.Publisher('CSEI/observer/eta', Vector3, queue_size=1)
    pub_nu = rospy.Publisher('CSEI/observer/nu', Vector3, queue_size=1)
    pub_bias = Publisher('CSEI/observer/eta', Vector3, queue_size=1)

def nodeEnd():
    node.destroy_node()