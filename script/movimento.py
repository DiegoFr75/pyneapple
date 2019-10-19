#!/usr/bin/env python
import tty
import sys
import termios
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from math import pi
from sensor_msgs.msg import Joy
from time import sleep
import termios
import time

# ESC para finalizar
# "+"  - aumenta a Velocidade
# "-" - diminui a Velocidade
# "w" - segue em frente
# "s" - re
# "q" - diagonal esquerda para frente
# "e" - diagonal direita para frente
# "a" - diagonal esquerda para tras
# "d" - diagonal direita para tras

def getGrau(grau):
	return (21*grau)/360

def bigger(A, B):
	if A > B:
		return A
	return B

class RosSelfDrive():
    # class attributes
	max_translational_speed = 5 # in [m/s]
	max_rotational_speed = 10 # in [rad/s]
	max_arms_rotational_speed = 0.52 # in [rad/s]

	# how to obtain these values? see Mandow et al. COMPLETE THIS REFERENCE
	var_lambda = 0.965
	wheel_radius = 0.1324
	ycir = 0.531

	# class constructor
	def __init__(self):

		# initializing some attributes
		self.x, self.y, self.z = (0,0,0)
		self.omega_left = 0
		self.omega_right = 0
		self.arm_front_rotSpeed = 0
		self.arm_rear_rotSpeed = 0

		self.axes_lin = 0
		self.axes_ang = 0

		self.axes_lin_saved = 1
		self.axes_ang_saved = 1

		self.trigger_left = 0
		self.trigger_right = 0

		# A Button
		self.button_L = 0
		# X Button
		self.button_R = 1
		
		self.rotate = 0
		self.drive = 1

		# computing the kinematic A matrix
		self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

		# sends a message to the user
		rospy.loginfo('ros_selfdrive node started')

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
		self.pub_arm = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)
		self.sub_gps = rospy.Subscriber("/sensor/gps", NavSatFix, self.getGpsData)

		self.direction_pub = rospy.Publisher("/direction", String, queue_size=10)
		self.direction_sub = rospy.Subscriber("/direction", String, self.moveRosi)

		self.arm_command_list = RosiMovementArray()
		self.traction_command_list = RosiMovementArray()

		rospy.spin()
	
	def moveRosi(self, msg):
		if(msg.data == "start"):
			self.rotateAntiClockwise(30)
			self.moveForward(152)
			self.stop()
			self.rotateClockwise(14)
			self.moveForward(224)
			self.stop()
			#inserir movimento robo fogo 1
			self.toca_fogo1()
			self.rotateAntiClockwise(6)
			self.moveForward(15)
			self.stop()
			self.rotateAntiClockwise(6)
			self.moveForward(180)
			self.stop()
			self.rotateClockwise(2)
			self.moveForward(20)
			self.rotateClockwise(2.5)
			self.moveForward(100)
			self.stop()
			#desvio de obstaculo 1
			self.moveForward(50)
			self.stop()
			self.moveForward(300)
			self.rotateAntiClockwise(2.5)
			self.moveForward(100)
			self.stop()
			#desvio obstaculo 2
			self.rotateClockwise(6)
			self.moveForward(147)
			self.stop()
			self.rotateAntiClockwise(6)
			self.moveForward(50)
			self.rotateAntiClockwise(3)
			self.moveForward(30)
			self.stop()
			#reta da escada
			self.moveForward(300)
			self.stop()
			self.rotateAntiClockwise(2)
			self.moveForward(300)
			self.stop()
			#chegando a escada
			self.moveForward(110)
			self.stop()
			#inserir codigo de subida de escada
			self.climbStairs()
			self.moveForward(100)
			self.moveBackward(85)
			#inserir codigo de descida
			self.prepareToGoToFloor()
			time.sleep(15)
			self.backToFloor()



		# print(msg.data)
		# print(self.x, self.y, self.z)
		# if(msg.data == "forward"):
		# 	self.moveForward(0)
		# elif(msg.data == "backward"):
		# 	self.moveBackward(0)
		# elif(msg.data == "stop"):
		# 	self.stop()
		# elif(msg.data == "left" and self.drive):
		# 	self.rotateAntiClockwise(0)
		# elif(msg.data == "right"):
		# 	self.rotateClockwise(3)
		# 	# self.rotateAntiClockwise(0)
		# elif(msg.data == "climb"):
		# 	self.climbStairs()
		
	def getGpsData(self,msg):
		precision = "%0.4f"
		self.x = float(precision % (msg.latitude))
		self.y = float(precision % (msg.longitude))
		self.z = float(precision % (msg.altitude))

		if(self.drive):
			self.direction_pub.publish("start")
		# if(self.x >= 0.5400 and self.x <= 0.5450):
		# 	self.direction_pub.publish("forward")
		# if(self.x <= -1.5127 and self.x >= -1.5200):
		# 	self.direction_pub.publish("left")
		# if((self.x >= -1.1850 and self.x <= -1.1800)):
		# 	self.direction_pub.publish("forward")

	def sendCommand(self):
		# mounting the lists
			for i in range(4):

				# ----- treating the traction commands
				traction_command = RosiMovement()

				# mount traction command list
				traction_command.nodeID = i+1

				# separates each traction side command
				if i < 2:
					traction_command.joint_var = self.omega_right
				else:
					traction_command.joint_var = self.omega_left

				# appending the command to the list
				self.traction_command_list.movement_array.append(traction_command)

				# ----- treating the arms commands		
				arm_command = RosiMovement()
		
				# mounting arm command list
				arm_command.nodeID = i+1
				
				# separates each arm side command
				if i == 0 or i == 2:
					arm_command.joint_var = self.arm_front_rotSpeed
				else:
					arm_command.joint_var = self.arm_rear_rotSpeed

				# appending the command to the list
				self.arm_command_list.movement_array.append(arm_command)

			# publishing
			self.pub_arm.publish(self.arm_command_list)		
			self.pub_traction.publish(self.traction_command_list)
	
	def assembleAndSendCommands(self):
		# treats triggers range
		self.trigger_left = ((-1 * self.trigger_left) + 1) / 2
		self.trigger_right = ((-1 * self.trigger_right) + 1) / 2

		# computing desired linear and angular of the robot
		vel_linear_x = self.max_translational_speed * self.axes_lin
		vel_angular_z = self.max_rotational_speed * self.axes_ang

		# -- computes traction command - kinematic math

		# b matrix
		b = np.array([[vel_linear_x],[vel_angular_z]])

		# finds the joints control
		x = np.linalg.lstsq(self.kin_matrix_A, b, rcond=-1)[0]

		# query the sides velocities
		if(self.rotate == 1): # Right
			value = bigger(np.deg2rad(x[0][0]), np.deg2rad(x[1][0]))
			self.omega_left = value
			self.omega_right = -value
		elif(self.rotate == 2): # Left
			value = bigger(np.deg2rad(x[0][0]), np.deg2rad(x[1][0]))
			self.omega_left = -value
			self.omega_right = value
		else:
			self.omega_right = np.deg2rad(x[0][0])
			self.omega_left = np.deg2rad(x[1][0])

		# -- computes arms command
		# front arms
		if self.button_R == 1:
			self.arm_front_rotSpeed = self.max_arms_rotational_speed * self.trigger_right
		else:
			self.arm_front_rotSpeed = -1 * self.max_arms_rotational_speed * self.trigger_right

		# rear arms
		if self.button_L == 1:
			self.arm_rear_rotSpeed = -1 * self.max_arms_rotational_speed * self.trigger_left
		else:
			self.arm_rear_rotSpeed = self.max_arms_rotational_speed * self.trigger_left
		
		self.sendCommand()

	def stopAndClose(self):
		self.axes_ang = 0
		self.axes_lin = 0
		self.trigger_right = 0
		self.trigger_left = 0
		self.assembleAndSendCommands()
		sys.exit()

	def moveFrontArmsUp(self, tempo):
		self.trigger_right = 30
		self.rotate = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stopFrontArms()
		
	def moveFrontArmsDown(self, tempo):
		self.trigger_right = -30
		self.rotate = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stopFrontArms()

	def stopFrontArms(self):
		self.trigger_right = 0
		self.rotate = 0
		self.assembleAndSendCommands()

	def moveRearArmsUp(self, tempo):
		self.trigger_left = 30
		self.rotate = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stopRearArms()

	def moveRearArmsDown(self, tempo):
		self.trigger_left = -30
		self.rotate = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stopRearArms()

	def stopRearArms(self):
		self.trigger_left = 0
		self.rotate = 0
		self.assembleAndSendCommands()
	
	def moveForward(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = 0
		self.rotate = 0
		self.drive = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stop()

	def moveBackward(self, tempo):
		self.axes_lin = -1*self.axes_lin_saved
		self.axes_ang = 0
		self.rotate = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stop()

	def rotateAntiClockwise(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = self.axes_ang_saved
		self.rotate = 2
		self.drive = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stop()

	def rotateClockwise(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = -1*self.axes_ang_saved
		self.rotate = 1
		self.drive = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stop()

	def stop(self):
		self.axes_lin = 0
		self.axes_ang = 0
		self.rotate = 0
		self.assembleAndSendCommands()

	def increaseSpeed(self):
		self.axes_lin_saved += 0.1
		if(self.axes_lin != 0):
			self.axes_lin = self.axes_lin_saved
		#print("Velocidade atual: "+str(self.max_translational_speed*self.axes_lin_saved)+'m/s')
		self.assembleAndSendCommands()

	def decreaseSpeed(self):
		self.axes_lin_saved -= 0.1
		if(self.axes_lin != 0):
			self.axes_lin = self.axes_lin_saved
		#print("Velocidade atual: "+str(self.max_translational_speed*self.axes_lin_saved)+'m/s')
		self.assembleAndSendCommands()

	def prepareToGoToFloor(self):
		self.moveRearArmsUp(getGrau(70))

	def backToFloor(self):
		self.moveRearArmsDown(getGrau(40))

	def climbStairs(self):
		# self.moveFrontArmsUp(getGrau(90))
		self.moveForward(0)

		self.moveFrontArmsUp(4)
		self.moveRearArmsDown(4)
	
		time.sleep(17)

		self.moveFrontArmsDown(15)
		time.sleep(5)

		# Descer braco de tras
		self.moveRearArmsUp(8)

		time.sleep(10)

		# print("bracos da frente Down")
		# self.moveFrontArmsDown(19)
		
		# print("bracos de tras Up")
		# self.moveRearArmsDown(2)
		time.sleep(4)
		self.moveFrontArmsDown(getGrau(120))

		self.moveRearArmsUp(1)

		for i in range(30):
			self.moveRearArmsDown(0.1)
			time.sleep(0.2)
		
		# print("6")
		# self.moveFrontArmsDown(4)
		self.moveRearArmsDown(0)
		time.sleep(25)
		self.moveRearArmsUp(getGrau(180))
	
	# funcoes do codigo das juntas adicionadas aqui
	def mexe_junta(self, position_list=[pi/2,0, 0, 0, pi/2, 0], max_cont=50):
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

	def toca_fogo1(self):
		self.mexe_junta()
		self.mexe_junta([pi/2,-0.225, 0, 0.3, pi/2, 0])
		self.mexe_junta([pi/2,-0.225, -pi/28, 0.3, pi/2, 0])
		self.mexe_junta([pi/2,-0.38, -pi/28, 0.3, pi/2, 0])
		self.mexe_junta([pi/2,-0.48, -pi/23, 0.3, pi/2, 0], max_cont=25)
		self.mexe_junta()
		self.mexe_junta([pi/2, 0, pi/1.77, 0, pi/2, 0], max_cont=25)
		self.mexe_junta([pi/2,-pi/2, pi/1.7, 0, pi/2, 0])
		self.mexe_junta([pi/2,-pi/2, pi/1.92, 0, pi/2, 0])
		self.mexe_junta([pi/2,-pi/2, pi/1.7, 0, pi/2, 0])
		self.mexe_junta()

	def toca_fogo2(self):
		self.mexe_junta([pi/2, 0, 0, 0, -pi/2, 0])
		self.mexe_junta([pi/2, 0.225, 0, -0.3, -pi/2, 0])
		self.mexe_junta([pi/2, 0.225, pi/3, -0.3, -pi/2, 0], max_cont=25)
		self.mexe_junta([pi/2, 0, 0, 0, -pi/2, 0])
		self.mexe_junta([pi/2, 0.174533, -0.523599, 0.349066, -pi/2, 0])
		self.mexe_junta([pi/2, pi/2, -pi/1.77, 0, -pi/2, 0])
		self.mexe_junta([pi/2, pi/2, -pi/2, 0, -pi/2, 0], max_cont=25)
		self.mexe_junta([pi/2, pi/2, -pi/1.77, 0, -pi/2, 0])
		self.mexe_junta([pi/2, 0, 0, 0, -pi/2, 0])

	# ---- Support Methods --------

	# -- Method for compute the skid-steer A kinematic matrix
	@staticmethod
	def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):
		# kinematic A matrix 
		matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
							[(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])
		return matrix_A
    
if __name__ == "__main__":
    rospy.init_node('rosi_selfdrive', anonymous = True)

    try:
		node_obj = RosSelfDrive()
    except Exception as erro:
		print(erro)
		pass
