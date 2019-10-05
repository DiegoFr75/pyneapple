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
        #print(hello_str)
        #print(cont)
        cont+=1
        rate.sleep()


def toca_fogo1():
    print("Zero")
    mexe_junta()
    print("Meio toque 1")
    mexe_junta([pi/2,-0.225, 0, 0.3, pi/2, 0])
    print("Meio toque 2")
    mexe_junta([pi/2,-0.225, -pi/28, 0.3, pi/2, 0])
    print("Meio toque 3")
    mexe_junta([pi/2,-0.38, -pi/28, 0.3, pi/2, 0])
    print("Toque alto")
    mexe_junta([pi/2,-0.48, -pi/23, 0.3, pi/2, 0], max_cont=25)
    print("Zero")
    mexe_junta()
    print("Meio toque 1")
    mexe_junta([pi/2, 0, pi/1.77, 0, pi/2, 0], max_cont=25)
    mexe_junta([pi/2,-pi/2, pi/1.7, 0, pi/2, 0])
    print("Toque baixo")
    mexe_junta([pi/2,-pi/2, pi/1.92, 0, pi/2, 0])
    mexe_junta([pi/2,-pi/2, pi/1.7, 0, pi/2, 0])
    print("Zero")
    mexe_junta()

def toca_fogo2():
    print("Zero")
    mexe_junta([pi/2, 0, 0, 0, -pi/2, 0])
    print("Meio toque")
    mexe_junta([pi/2, 0.225, 0, -0.3, -pi/2, 0])
    print("Toque alto")
    mexe_junta([pi/2, 0.225, pi/3, -0.3, -pi/2, 0], max_cont=25)
    print("Zero")
    mexe_junta([pi/2, 0, 0, 0, -pi/2, 0])
    print("Meio toque")
    mexe_junta([pi/2, 0.174533, -0.523599, 0.349066, -pi/2, 0])
    print("Meio toque 2")
    mexe_junta([pi/2, pi/2, -pi/1.77, 0, -pi/2, 0])
    print("Toque baixo")
    mexe_junta([pi/2, pi/2, -pi/2, 0, -pi/2, 0], max_cont=25)
    print("Pos toque")
    mexe_junta([pi/2, pi/2, -pi/1.77, 0, -pi/2, 0])
    print("Zero")
    mexe_junta([pi/2, 0, 0, 0, -pi/2, 0])

if __name__ == '__main__':
    try:
        rospy.init_node('mexe_junta', anonymous=True)
        toca_fogo1()
    except rospy.ROSInterruptException:
        pass
