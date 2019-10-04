#!/usr/bin/env python
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
import time

class AutonomousClass():
	# class attributes
	max_translational_speed = 5 # in [m/s]
	max_rotational_speed = 10 # in [rad/s]
	max_arms_rotational_speed = 0.52 # in [rad/s]

	# how to obtain these values? see Mandow et al. COMPLETE THIS REFERENCE
	var_lambda = 0.965
	wheel_radius = 0.1324
	ycir = 0.531

	def __init__(self):

		# initializing some attributes
		self.omega_left = 0
		self.omega_right = 0
		self.axes_ang = 0
		self.axes_lin = 0
		self.axes_lin_saved = 1
		self.axes_ang_saved = 1
		self.drive = 1

		# computing the kinematic A matrix
		self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

		# sends a message to the user
		rospy.loginfo('Rosi Movement started')

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
		node_sleep_rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			self.traction_command_list = RosiMovementArray()
			if(self.drive):
				self.autonomousDrive()
				pass
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

		self.pub_traction.publish(self.traction_command_list)

	def calculateCommand(self):

		# treats triggers range
		#trigger_left = ((-1 * trigger_left) + 1) / 2
		#trigger_right = ((-1 * trigger_right) + 1) / 2

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

		self.sendCommand()

	def moveForward(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = 0
		self.calculateCommand()
		if(tempo):
			time.sleep(tempo)
			print("oi3")
			self.stop()

	def moveBackward(self, tempo):
		self.axes_lin = -1*self.axes_lin_saved
		self.axes_ang = 0
		self.calculateCommand()
		if(tempo):
			time.sleep(tempo)
			self.stop()

   	def rotateAntiClockwise(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = self.axes_ang_saved
		self.calculateCommand()
		if(tempo):
			time.sleep(tempo)
			self.stop()

   	def rotateClockwise(self, tempo):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = -1*self.axes_ang_saved
		self.calculateCommand()
		if(tempo):
			time.sleep(tempo)
			self.stop()

   	def stop(self):
		self.axes_lin = 0
		self.axes_ang = 0
		self.calculateCommand()

	def autonomousDrive(self):
		self.drive = 0
		self.moveForward(5)
		print("oi")
		#if(obstaculo #alterar o valor de obstaculo em algum canto rs):
			#self.stop()
			#rotacionar pra uma direcao
			#dar re?
			#self.autonomousDrive() #nao garanto qe ta certo, so tem que andar de novo


	@staticmethod
	def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):
		# kinematic A matrix 
		matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
								[(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])
		return matrix_A

	
if __name__ == "__main__":
    rospy.init_node('rosi_movement', anonymous = True)

    try:
		node_obj = AutonomousClass()

    except Exception as erro:
        print(erro)
        pass



