import rospy
from lib import odometry, ps4, Udata
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math


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





def loop():


    leftThrustAngle = math.atan2(ps4.lStickY, ps4.lStickX)
    rightThrustAngle = math.atan2(ps4.rStickY, ps4.rStickX)
    leftThrustLength = math.sqrt(ps4.lStickX ** 2 + ps4.lStickY ** 2)
    rightThrustLength = math.sqrt(ps4.rStickX ** 2 + ps4.rStickY ** 2)
    bowThrustLengt = (ps4.R2A - ps4.L2A)

    Udata.publish([leftThrustLength, rightThrustLength, bowThrustLengt, leftThrustAngle, rightThrustAngle])
