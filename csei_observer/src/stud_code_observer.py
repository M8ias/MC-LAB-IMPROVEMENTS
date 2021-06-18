import rospy
from lib import observer, Udata


def loop():

    print(observer.getQualisysOdometry())
    print(Udata.getU())

    return 0