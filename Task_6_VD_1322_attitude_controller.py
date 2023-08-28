#!/usr/bin/env python
'''
# Team ID:          VD#1322
# Theme:            Vitarana Drone
# Author List:      Prakhar Agrawal, Abha Porwal, Apoorv Nema, Kanak Verma
# Filename:         Task_6_VD_1322_position_controller
# Functions:        imu_callback,drone_command_callback,roll_set_pid,pitch_set_pid,yaw_set_pid,alt_error,roll_error,pitch_error,pid,limit
# Global variables: None
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import tf, math


class Edrone():

    	"""Edrone is a class which defines the whole control system for an attitude controller of our drone"""

    	def __init__(self):

    	    	rospy.init_node('attitude_controller')  # Initializing ros node with name drone_control
	
    		# Initializing all the variables that are going to be used further

    	    	self.error = [0, 0, 0, 0]
    	    	self.last_err = [0, 0, 0, 0]  #variables for storing previous errors in each axis
    	    	self.err_sum = [0, 0, 0, 0]
    	    	self.d_err = [0, 0, 0, 0]

    		# Iniitializing the maximum and minimum value for all the propellers' speed

    	    	self.max_value = 1024
    	    	self.min_value = 0

    		# Miscellaneous Values

    	    	self.throttle = 0
    	    	self.throttle_conv = 0
    	    	self.last_time = 0  
    	    	self.out_roll = 0
    	    	self.set_altitude = 0 
    	    	self.set_latitude = 0
    	    	self.set_longitude = 0
    	    	self.aux = 1000

    		# This corresponds to your current orientation of eDrone in quaternion format

    	    	self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

    		# This corresponds to your current orientation of eDrone converted in euler angles form

    	    	self.drone_orientation_euler = [0.0, 0.0, 0.0]

    		# This is the setpoint that will be received from the drone_command in the range from 1000 to 2000 [r_setpoint, p_setpoint, y_setpoint]

    		self.setpoint_cmd = [0.0, 0.0, 0.0]

    		# The setpoint of orientation in euler angles at which you want to stabilize the drone [r_setpoint, p_psetpoint, y_setpoint]

    		self.setpoint_euler = [0.0, 0.0, 0.0]

    		# Declaring pwm_cmd of message type prop_speed and initializing values

    		self.pwm_cmd = prop_speed()
    		self.pwm_cmd.prop1 = 0.0
    		self.pwm_cmd.prop2 = 0.0
    		self.pwm_cmd.prop3 = 0.0
    		self.pwm_cmd.prop4 = 0.0

    		# After tuning and computing corresponding PID parameters are taken as their default values in [r, p ,y] respectively

    		self.Kp = [157.2, 157.2, 1598.4]
    		self.Ki = [0, 0, 0]
    		self.Kd = [30, 40, 124.5]

    		# This is the sample time in which we need to run pid. 

        	self.sample_time = 0.02  # in seconds

    		# Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error , /zero_error

        	self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size = 1)
        	self.value_pub2 = rospy.Publisher('/roll_error', Float32, queue_size = 1)
        	self.value_pub3 = rospy.Publisher('/pitch_error', Float32, queue_size = 1)
        	self.value_pub4 = rospy.Publisher('/yaw_error', Float32, queue_size = 1)
        	self.value_pub5 = rospy.Publisher('/zero_error', Float32, queue_size = 1)


    		# Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw, /alt_error, /lat_error, /long_error

        	rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        	rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        	rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        	rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        	rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        	rospy.Subscriber('/altitude_pwm_out', Float32, self.alt_error)
        	rospy.Subscriber('/lat_pwm_out', Float32, self.roll_error)
        	rospy.Subscriber('/long_pwm_out', Float32, self.pitch_error)

    	def imu_callback(self, msg):
		'''
		Purpose:
		---
		Imu callback function which gets executed each time when imu publishes /edrone/imu/data to get the coordinates

		Input Arguments:
		---
		msg :  [ Imu ]
		Imu is inertial measurement unit which gives us drone's orientation

		Returns:
		---
		None
		'''

        	self.drone_orientation_quaternion[0] = msg.orientation.x
        	self.drone_orientation_quaternion[1] = msg.orientation.y
        	self.drone_orientation_quaternion[2] = msg.orientation.z
        	self.drone_orientation_quaternion[3] = msg.orientation.w

    	def drone_command_callback(self, msg):
		'''
		Purpose:
		---
		Drone command callback function which gets executed each time when we need the setpoints i.e. roll,pitch and yaw etc.
		
		Input Arguments:
		---
		msg :  [ edrone_cmd ]
		getting the drone's roll, pitch and yaw
		
		Returns:
		---
		None
		'''

        	self.setpoint_cmd[0] = msg.rcRoll
        	self.setpoint_cmd[1] = msg.rcPitch
        	self.setpoint_cmd[2] = msg.rcYaw
        	self.throttle = msg.rcThrottle
        	self.aux = msg.aux1

    	def roll_set_pid(self, roll):
		'''
		Purpose:
		---
		Callback function for /pid_tuning_roll

		Input Arguments:
		---
		roll :  [ PidTune ]
		Tuning values of drone's roll

		Returns:
		---
		None
		'''

        	self.Kp[0] = roll.Kp * 0.06  
        	self.Ki[0] = roll.Ki * 0.008
        	self.Kd[0] = roll.Kd * 0.3

    	def pitch_set_pid(self, pitch):
	     	'''
		Purpose:
		---
		Callback function for /pid_tuning_pitch

		Input Arguments:
		---
		pitch :  [ PidTune ]
		Tuning values of drone's pitch

		Returns:
		---
		None
		'''

        	self.Kp[1] = pitch.Kp * 0.06  
        	self.Ki[1] = pitch.Ki * 0.008
        	self.Kd[1] = pitch.Kd * 0.3

    	def yaw_set_pid(self, yaw):
		'''
		Purpose:
		---
		Callback function for /pid_tuning_yaw

		Input Arguments:
		---
		yaw :  [ PidTune ]
		Tuning values of drone's yaw

		Returns:
		---
		None
		'''

        	self.Kp[2] = yaw.Kp * 0.6
        	self.Ki[2] = yaw.Ki * 0.008
        	self.Kd[2] = yaw.Kd * 0.3

    	def alt_error(self, msg):
	      	'''
		Purpose:
		---
		Callback function for getting the data from msg file for altitude

		Input Arguments:
		---
		msg :  [ Float32 ]
		error value for altitude

		Returns:
		---
		None
		'''

        	self.set_altitude = msg.data

    	def roll_error(self, msg):
	        '''
		Purpose:
		---
		Callback function for getting the data from msg file for roll

		Input Arguments:
		---
		msg :  [ Float32 ]
		error value for roll

		Returns:
		---
		None
		'''

        	self.set_latitude = msg.data

    	def pitch_error(self, msg):
	    	'''
		Purpose:
		---
		Callback function for getting the data from msg file for pitch

		Input Arguments:
		---
		msg :  [ Float32 ]
		error value for pitch

		Returns:
		---
		None
		'''

        	self.set_longitude = msg.data

    	def pid(self):
		'''
		Purpose:
		---
		pid function to execute our whole control system for an attitude controller

		Input Arguments:
		---
		None

		Returns:
		---
		None
		'''

    		# Getting the current time and calculating the time required in each iteration

        	self.now = float(rospy.get_time())
        	time_change = self.now - self.last_time
                                                                                   
    		# Converting quaternion to euler angles

        	(self.drone_orientation_euler[0], self.drone_orientation_euler[1], 
		self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], 
		self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

    		# Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis

        	self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        	self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        	self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
	
    		# Converting the range of 1000 to 2000 to 0 to 1024 for throttle

        	self.throttle_conv = self.throttle * 1.024 - 1024
	
    		# Computing error(for proportional) in each axis [r,p,y] respectively

        	self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[1]
        	self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[0]
        	self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]
    		# Computing the sum of errors (for integral) in each axis

        	self.err_sum[0] += self.error[0] * time_change
        	self.err_sum[1] += self.error[1] * time_change
        	self.err_sum[2] += self.error[2] * time_change

    		# Computing change in error (for derivative) in each axis

        	if time_change > 0: #To avoid division with zero

            	self.d_err[0] = (self.error[0] - self.last_err[0]) / time_change
            	self.d_err[1] = (self.error[1] - self.last_err[1]) / time_change
            	self.d_err[2] = (self.error[2] - self.last_err[2]) / time_change

    		# Calculating the pid output required for each axis i.e. out_roll, out_pitch, out_yaw

	    	self.out_roll = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.err_sum[0]) + (self.Kd[0] * self.d_err[0])
        	self.out_pitch = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.err_sum[1]) + (self.Kd[1] * self.d_err[1])
        	self.out_yaw = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.err_sum[2]) + (self.Kd[2] * self.d_err[2])

    		# Computing the pwm for each propeller with the help of speed mixing algorithm using previously calculated roll, pitch and yaw
    
        	if(self.aux == 1500):

            	self.pwm_cmd.prop1 = self.limit(self.throttle_conv - self.out_roll + self.out_pitch - self.out_yaw + self.set_altitude - self.set_latitude + self.set_longitude)
            	self.pwm_cmd.prop2 = self.limit(self.throttle_conv - self.out_roll - self.out_pitch + self.out_yaw + self.set_altitude - self.set_latitude - self.set_longitude)
            	self.pwm_cmd.prop3 = self.limit(self.throttle_conv + self.out_roll - self.out_pitch - self.out_yaw + self.set_altitude + self.set_latitude - self.set_longitude)
            	self.pwm_cmd.prop4 = self.limit(self.throttle_conv + self.out_roll + self.out_pitch + self.out_yaw + self.set_altitude + self.set_latitude + self.set_longitude)

    		# Updating previous errors and last time

        	self.last_time = self.now
        	self.last_err[0] = self.error[0]
        	self.last_err[1] = self.error[1]
        	self.last_err[2] = self.error[2]
	
    		# Publishing all the errors for tuning the PID including pwm_command

        	self.pwm_pub.publish(self.pwm_cmd)
        	self.value_pub2.publish(self.error[0])
        	self.value_pub3.publish(self.error[1])
        	self.value_pub4.publish(self.error[2])
        	self.value_pub5.publish(0)
        
    	def limit(self, input):
	      	'''
		Purpose:
		---
		Limiting the output values so that they don't reach upto a limit

		Input Arguments:
		---
		input :  [ Float32 ]
		Value that has to be limited

		Returns:
		---
		input :  [ Float32 ]
		return the limited value
		'''

        	if input > self.max_value:

        	    	return self.max_value

        	if input < self.min_value:

        	    	return self.min_value

        	else:

        	    	return input
        

if __name__ == '__main__':

    	# Creating an instance of the class

    	e_drone = Edrone() 

    	# specifying the rate in Hz based upon PID sampling time

    	r = rospy.Rate(1 / (e_drone.sample_time))
	
    	while not rospy.is_shutdown():

        	e_drone.pid()
        	r.sleep()


