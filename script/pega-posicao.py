#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from math import pi
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import Joy
from time import sleep

def callback(data):
    # rospy.loginfo(rospy.get_caller_id()+'i heard %s' %data.data)
    z = data.twist.linear.z
    if z >= 0:
        print(z)

def get_pos():
    rospy.init_node('pega_pos', anonymous=True)
    sub = rospy.Subscriber('/ur5/forceTorqueSensorOutput', TwistStamped, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        get_pos()
    except rospy.ROSInterruptException:
        pass
