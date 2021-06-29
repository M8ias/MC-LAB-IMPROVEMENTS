import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
import numpy as np
from matplotlib.animation import FuncAnimation
from Kinematics import quat2eul

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 10000)
        self.ax.set_ylim(-7, 7)
        return self.ln

    def getYaw(self, pose):
        euler = quat2eul(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        yaw = euler[2] 
        return yaw   

    def odom_callback(self, msg):
        yaw_angle = self.getYaw(msg.pose.pose)
        self.y_data.append(yaw_angle)
        x_index = len(self.x_data)
        self.x_data.append(x_index+1)

    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln


rospy.init_node('visualiser')
vis = Visualiser()
sub = rospy.Subscriber('/qualisys/CSEI/odom', Odometry, vis.odom_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True)