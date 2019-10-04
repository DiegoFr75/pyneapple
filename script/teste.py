#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image


def callback(msg):
	#rospy.loginfo("\n")
	with open("teste.txt","w") as file:
		file.write(msg.data)
#rospy.loginfo(msg.reading[360])

rospy.init_node('scan_values')
sub = rospy.Subscriber('/sensor/kinect_rgb', Image, callback)
rospy.spin()


