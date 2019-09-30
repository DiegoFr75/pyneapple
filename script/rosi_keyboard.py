#!/usr/bin/env python
import tty
import sys
import termios
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
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

class RosiKeyboardClass():
    # class attributes
    max_translational_speed = 20 # in [m/s]
    max_rotational_speed = 50 # in [rad/s]
    max_arms_rotational_speed = 0.7 # in [rad/s]

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

		self.orig_settings = termios.tcgetattr(sys.stdin)
		tty.setcbreak(sys.stdin)

		# computing the kinematic A matrix
		self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

		# sends a message to the user
		rospy.loginfo('Rosi_joy node started')

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
		self.pub_arm = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		# eternal loop (until second order)
		pressedKey = 0
		while not rospy.is_shutdown():
			# self.callback_Joy()
			self.arm_command_list = RosiMovementArray()
			self.traction_command_list = RosiMovementArray()

			# Get pressed key
			pressedKey = sys.stdin.read(1)[0]

			if(pressedKey):
				self.selectCommand(pressedKey)

			# sleeps for a while
			node_sleep_rate.sleep()

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)

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
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
		self.axes_ang = 0
		self.axes_lin = 0
		self.trigger_right = 0
		self.trigger_left = 0
		self.assembleAndSendCommands()
		sys.exit()

    def moveFrontArmsUp(self):
		self.trigger_right = 30
		self.assembleAndSendCommands()

    def moveFrontArmsDown(self):
		self.trigger_right = -30
		self.assembleAndSendCommands()

    def stopFrontArms(self):
		self.trigger_right = 0
		self.assembleAndSendCommands()

    def moveRearArmsUp(self):
		self.trigger_left = 30
		self.assembleAndSendCommands()

    def moveRearArmsDown(self):
		self.trigger_left = -30
		self.assembleAndSendCommands()

    def stopRearArms(self):
		self.trigger_left = 0
		self.assembleAndSendCommands()
	
    def moveForward(self):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = 0
		self.assembleAndSendCommands()

    def moveBackward(self):
		self.axes_lin = -1*self.axes_lin_saved
		self.axes_ang = 0
		self.assembleAndSendCommands()

    def rotateAntiClockwise(self):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = self.axes_ang_saved
		self.assembleAndSendCommands()

    def rotateClockwise(self):
		self.axes_lin = self.axes_lin_saved
		self.axes_ang = -1*self.axes_ang_saved
		self.assembleAndSendCommands()

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
		# self.moveRearArmsUp()
		# time.sleep(0.5)
		# self.stopRearArms()

		# self.moveFrontArmsDown()
		# time.sleep(0.5)
		# self.stopFrontArms()
		self.moveFrontArmsUp()
		time.sleep(1.5)
		self.stopFrontArms()

		self.moveForward()
		time.sleep(6)

		self.moveFrontArmsDown()
		time.sleep(2)
		self.stopFrontArms()

		self.moveRearArmsUp()
		time.sleep(1)
		self.stopRearArms()
		time.sleep(6)
		self.moveRearArmsDown()
		time.sleep(1.5)
		self.stopRearArms()

		self.stop()

		
    def selectCommand(self, pressedKey):	
		if(pressedKey == chr(27)):
			self.stopAndClose()
		elif(pressedKey == "w"):
			self.moveForward()
		elif(pressedKey == "s"):
			self.moveBackward()
		elif(pressedKey == "q"):
			self.rotateAntiClockwise()
		elif(pressedKey == "e"):
			self.rotateClockwise()
		elif(pressedKey == "r"):
			self.stop()
		elif(pressedKey == "f"):
			self.climbStairs()
		elif(pressedKey == '+'):
			self.increaseSpeed()
		elif(pressedKey == '-'):
			self.decreaseSpeed()
	
    # ---- Support Methods --------

    # -- Method for compute the skid-steer A kinematic matrix
    @staticmethod
    def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):

        # kinematic A matrix 
        matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
                            [(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])

        return matrix_A
    
if __name__ == "__main__":
    rospy.init_node('rosi_keyboard', anonymous = True)

    try:
		node_obj = RosiKeyboardClass()
    except Exception as erro:
        print(erro)
        pass
