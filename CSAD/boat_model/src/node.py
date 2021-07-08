#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def handle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, -msg.pose.pose.position.y, msg.pose.pose.position.z), (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, -msg.pose.pose.orientation.z, msg.pose.pose.orientation.w), rospy.Time.now(),"base_link", "odom")

if __name__ == '__main__':
    rospy.init_node('tf_test_node')
    rospy.Subscriber('qualisys/CSAD/odom',Odometry, handle_pose)
    rospy.spin()