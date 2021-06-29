#!/usr/bin/env python3

import rospy 

from dynamic_reconfigure.server import Server
from parameter_server_basics.cfg import gainsConfig


def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {L1}, {L2}, {L3}, {Kp}, {Kd}, {Ki}, {mu}, {U_ref}""".format(**config))
    return config

if __name__=="__main__":
    rospy.init_node("parameter_server_basics", anonymous = False)
    srv = Server(gainsConfig, callback)
    rospy.spin()