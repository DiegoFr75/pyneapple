#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

def callback(msg):
	l=len(msg.data)
	x=0	
	y=(l/3)-1
	z=(2*l/3)-1
	matrix = []
	for i in range(l/3):
		aux = []
		aux.append(ord(msg.data[x]))
		aux.append(ord(msg.data[y]))
		aux.append(ord(msg.data[z]))
		matrix.append(aux)
		x+=1
		y+=1
		z+=1
	
	rospy.loginfo(matrix[0])
	

rospy.init_node('scan_values')
sub = rospy.Subscriber('/sensor/velodyne', PointCloud2, callback)
rospy.spin()

