#!/usr/bin/env python3
import rospy
from stud_code_controll import loop
from joy_listener import controllNodeInit, nodeEnd




if __name__ == '__main__':

    controllNodeInit()
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        loop()
        r.sleep()
    
    nodeEnd()