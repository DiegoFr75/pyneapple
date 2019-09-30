import cv2
import numpy as np
import os

def ta_pegando_fogo(img):
	blur = cv2.GaussianBlur(img, (21, 21), 0)
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

	lower = [21, 50, 50]
	upper = [50, 255, 255]
	lower = np.array(lower, dtype="uint8")
	upper = np.array(upper, dtype="uint8")
	mask = cv2.inRange(hsv, lower, upper)



	output = cv2.bitwise_and(img, hsv, mask=mask)

	return output

def get_fire_position(img):
	orig = img
	img = ta_pegando_fogo(img)
	ret, threshed_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
	                127, 255, cv2.THRESH_BINARY)

	contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if contours != None:
		biggest_box = (x, y, w, h) = cv2.boundingRect(contours[0])
		first_in = True
		for c in contours:
			if first_in:
				first_in = False
				continue
			x, y, w, h = cv2.boundingRect(c)
			if(w*h < 1000):
				continue
			if (w*h > biggest_box[2]*biggest_box[3]):
				biggest_box = (x, y, w, h)
  
            # draw a green rectangle to visualize the bounding rect
			# cv2.rectangle(orig, (x, y), (x+w, y+h), (0, 255, 0), 2)
			# cv2.imshow('output', orig)
			# cv2.waitKey(0)

	return biggest_box



#### EXEMPLO:

img = cv2.imread('fogo.png')#trocar por leitura da camera
box = get_fire_position(img)#recebe coordenadas do posicionamento do fogo na imagem
print(box)