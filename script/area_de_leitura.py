#! /usr/bin/env python

import rospy
from rosi_defy.msg import HokuyoReading

def callback(msg):
		for i in range(277,347):
			rospy.loginfo((msg.reading[i]))
			
		rospy.loginfo("comecando outro\n")			


		#rospy.loginfo(msg.reading[360])

rospy.init_node('scan_values')
sub = rospy.Subscriber('/sensor/hokuyo', HokuyoReading, callback)
rospy.spin()


