#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from math import pi
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import Joy
from time import sleep




def mexe_junta(position_list=[pi/2,0, 0, 0, pi/2, 0], max_cont=50):
    pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)
    # rospy.init_node('mexe_junta', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    
    cont = 0
    while not rospy.is_shutdown() and cont<max_cont:
        hello_str = "hello world %s" % rospy.get_time()
        
        arg = ManipulatorJoints()
        arg.header.seq = 1
        arg.header.stamp.secs = 0.2 
        arg.header.stamp.nsecs = 1000

        # rospy.loginfo(position)

        position = pi/2

        arg.joint_variable = position_list
        pub.publish(arg.header, arg.joint_variable)
        print(hello_str)
        print(cont)
        cont+=1
        rate.sleep()





if __name__ == '__main__':
    try:
        rospy.init_node('mexe_junta', anonymous=True)
        mexe_junta()
        mexe_junta([pi/2,-0.225, -pi/10, 0.3, pi/2, 0], max_cont=25)
        mexe_junta()
        mexe_junta([pi/2, 0, pi/1.77, 0, pi/2, 0], max_cont=25)
        mexe_junta([pi/2,-pi/2, pi/1.77, 0, pi/2, 0])
        mexe_junta()
    except rospy.ROSInterruptException:
        pass
