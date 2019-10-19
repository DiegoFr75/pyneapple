#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from math import pi
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import Joy
from time import sleep
import cv2
from cv_bridge import CvBridge, CvBridgeError


class FireDector:
	def __init__(self):
		self.bridge = CvBridge()

		rospy.init_node('FireDetector', anonymous=True)

		rospy.loginfo('FireDetector node started')

		self.subCamera = rospy.Subscriber('/sensor/ur5toolCam', Image, self.callback)
		self.pubFire = rospy.Publisher('/fire', String, queue_size = 1)
		
		rospy.spin()
		

	def ta_pegando_fogo(self, imagem): # filtro para destacar a presenca de fogo
		blur = cv2.GaussianBlur(imagem, (21, 21), 0) # Suavizacao de Gauss
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) # Transformacao de BGR para HSV
		
		# limiares de corte de cores para mascara de fogo
		lower = [21, 50, 50]
		upper = [50, 255, 255]
		lower = np.asarray(lower)
		upper = np.asarray(upper)

		# mascara
		mask = cv2.inRange(hsv, lower, upper)


		# imagem filtrada
		output = cv2.bitwise_and(imagem, hsv, mask=mask)

		ret, threshed_img = cv2.threshold(cv2.cvtColor(output, cv2.COLOR_BGR2GRAY),127, 255, cv2.THRESH_BINARY)
		
		contours = cv2.findContours(threshed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		# print(contours[1])
		# cv2.imwrite('teste.jpg', contours)	
		if contours[1] != None:
			biggest_box = (x, y, w, h) = cv2.boundingRect(contours[1][0])
			first_in = True
			for c in contours[1]:
				if first_in:
					first_in = False
					continue
				
				x, y, w, h = cv2.boundingRect(c)
				if(w*h < 50000):
					continue
				if (w*h > biggest_box[2]*biggest_box[3]):
					biggest_box = (x, y, w, h)

				# draw a green rectangle to visualize the bounding rect
				# cv2.rectangle(imagem, (x, y), (x+w, y+h), (0, 255, 0), 2)
				# cv2.circle(imagem, (x+w//2,y+w//2), 2, (0,0,255), 2)
				# cv2.imwrite('reginaldo.jpg', imagem)
				# print(biggest_box)
				return x+w//2

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print(e)
	
		
		cv_image = cv2.flip(cv_image,1)
		fire_center = self.ta_pegando_fogo(cv_image)
		
		# Publicar aqui como string
		# self.pubFire.publish()


if __name__ == '__main__':
    try:
        fireDetector = FireDector()
    except rospy.ROSInterruptException:
        pass
