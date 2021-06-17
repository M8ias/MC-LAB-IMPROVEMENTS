import rospy
from joy_listener import controller, getControllerState, getObserverOdometry, publishU, U
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


ps4 = controller()
shipU = U()

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

# odometry.pose.pose.position.x
# odometry.pose.pose.position.y
# odometry.pose.pose.position.z
# odometry.pose.pose.orientation.x
# odometry.pose.pose.orientation.y
# odometry.pose.pose.orientation.z
# odometry.pose.pose.orientation.w
# odometry.twist.twist.linear.x
# odometry.twist.twist.linear.y
# odometry.twist.twist.linear.z
# odometry.twist.twist.angular.x
# odometry.twist.twist.angular.y
# odometry.twist.twist.angular.z

# shipU.leftRotorThrust
# shipU.rightRotorThrust
# shipU.bowRotorThrust
# shupU.leftRotorAngle
# shipU.rightRotorAngle




def loop():

    ps4 = getControllerState()
    odometry = getObserverOdometry()
    shipU.leftRotorThrust = 0.0
    shipU.rightRotorThrust = 0.0
    shipU.bowRotorThrust = 0.0
    shipU.leftRotorAngle = 0.0
    shipU.rightRotorAngle = 0.0

    publishU(shipU)
    print(odometry.pose.pose.position.x)
    print(ps4.R2A)
