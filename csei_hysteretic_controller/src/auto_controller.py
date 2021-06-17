#!/usr/bin/env python3
import rospy
from stud_code_controll import loop
from lib import controllNodeInit, nodeEnd
import yaml
import os

cwd = os.getcwd()
with open(r"{0}/src/csei_hysteretic_controller/src/params.yaml".format(cwd)) as file:
    params = yaml.load(file, Loader=yaml.FullLoader)


if __name__ == '__main__':

    controllNodeInit()
    r = rospy.Rate(params["runfrequency"])

    while not rospy.is_shutdown():
        loop()
        r.sleep()
    
    nodeEnd()