#!/usr/bin/env python
import tty
import sys
import termios
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints

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
		self.drive = 1 

		# computing the kinematic A matrix
		self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

		# sends a message to the user
		rospy.loginfo('ros_selfdrive node started')

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
		self.pub_arm = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			self.arm_command_list = RosiMovementArray()
			self.traction_command_list = RosiMovementArray()
			print("vai")
			if(self.drive):
				self.autonomousDrive(self.drive)
				#self.drive = 0

			# sleeps for a while
			node_sleep_rate.sleep()

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
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stopFrontArms()
		
	def moveFrontArmsDown(self, tempo):
		self.trigger_right = -30
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stopFrontArms()

	def stopFrontArms(self):
		self.trigger_right = 0
		self.assembleAndSendCommands()

	def moveRearArmsUp(self, tempo):
		self.trigger_left = 30
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stopRearArms()

	def moveRearArmsDown(self, tempo):
		self.trigger_left = -30
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stopRearArms()

	def stopRearArms(self):
		self.trigger_left = 0
		self.assembleAndSendCommands()
	
	def moveForward(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stop()

	def moveBackward(self, tempo):
		self.axes_lin = -1*self.axes_lin_saved
		self.axes_ang = 0
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stop()

	def rotateAntiClockwise(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = -1*self.axes_ang_saved
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stop()

	def rotateClockwise(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = self.axes_ang_saved
		self.assembleAndSendCommands()
		if(tempo):
			rospy.sleep(tempo)
			self.stop()

	def stop(self):
		self.axes_lin = 0
		self.axes_ang = 0
		self.assembleAndSendCommands()

	def increaseSpeed(self):
		self.axes_lin_saved += 0.1
		if(self.axes_lin != 0):
			self.axes_lin = self.axes_lin_saved
		print("Velocidade atual: "+str(self.max_translational_speed*self.axes_lin_saved)+'m/s')
		self.assembleAndSendCommands()

	def decreaseSpeed(self):
		self.axes_lin_saved -= 0.1
		if(self.axes_lin != 0):
			self.axes_lin = self.axes_lin_saved
		print("Velocidade atual: "+str(self.max_translational_speed*self.axes_lin_saved)+'m/s')
		self.assembleAndSendCommands()
	
	def climbStairs(self):
		self.moveForward(0)
		self.moveFrontArmsUp(4)
	
		rospy.sleep(14.7)
		self.moveFrontArmsDown(15)
		
		# Descer braco de tras
		self.moveRearArmsUp(4)

		rospy.sleep(10)
		rospy.sleep(3)
		self.moveRearArmsUp(1)

		for i in range(30):
			self.moveRearArmsDown(0.1)
			rospy.sleep(0.2)
	
		self.moveRearArmsDown(0)
		rospy.sleep(25)
		self.moveFrontArmsDown(getGrau(50))
		self.moveRearArmsUp(getGrau(90))
		self.stopAndClose()

	def selectCommand(self, pressedKey):	
		if(pressedKey == chr(27)):
			self.stopAndClose()
		elif(pressedKey == "w"):
			self.moveForward(0)
		elif(pressedKey == "s"):
			self.moveBackward(0)
		elif(pressedKey == "q"):
			self.rotateAntiClockwise(0)
		elif(pressedKey == "e"):
			self.rotateClockwise(0)
		elif(pressedKey == "r"):
			self.stop()
		elif(pressedKey == "f"):
			self.climbStairs()
		elif(pressedKey == '+'):
			self.increaseSpeed()
		elif(pressedKey == '-'):
			self.decreaseSpeed()

	def autonomousDrive(self, drive):
		#self.drive = 0
		#self.moveBackward(4)
		#self.stop()
		#self.rotateAntiClockwise(45)
		# self.moveForward(15)
		# self.climbStairs()
		self.rotateClockwise(5)
		# self.climbStairs()
		#self.moveForward(15)
		# self.rotateAntiClockwise(10)
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
