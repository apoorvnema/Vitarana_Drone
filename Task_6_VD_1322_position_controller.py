#!/usr/bin/env python
'''
# Team ID:          VD#1322
# Theme:            Vitarana Drone
# Author List:      Prakhar Agrawal, Abha Porwal, Apoorv Nema, Kanak Verma
# Filename:         Task_6_VD_1322_position_controller
# Functions:        bottom_sense, marker_data, range_finder,edrone_gps, set_altitude,set_latitude, 
					set_longitude, follow_wall_go_front, follow_wall_go_back, follow_wall_go_left, 
					follow_wall_go_right, go_to_pos, arm, disarm, lat_to_x, long_to_y, x_to_lat, 
					y_to_long, new_bottom_func, pos_check, truncate, pick_box, drop_box
# Global variables: init_flag, init_lat, init_long, init_alt, grip_res, coord_x, coord_y, lat,lat_set
'''

# Importing the required libraries

from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix,LaserScan
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import Float32, Int8
import rospy,rospkg
import pandas as pd
import os, math

# Initializing some global variables

pubg = -1
rospack = rospkg.RosPack()
rospack.list()
init_flag = 0
init_lat = 0
init_long = 0
init_alt = 0
grip_res = 3

# Miscellaneous variables

coord_x = 0
coord_y = 0
lat = 0
lat_set = 0

# Defining Function for calling gripper service

# Defining function to grab the box
def attach():
	'''
	Purpose:
	---
	This function is used to activate gripper and if the box gets attached with drone it checks value of result and change the global variable grip_res=1

	Input Arguments:
	---
	None

	Returns:
	---
	None
	'''

	req = 0
	resp = 0
	activate = 0
	rospy.wait_for_service('/edrone/activate_gripper')
	activate = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
	req = GripperRequest(True)
	resp = activate(req)
	GripperResponse(resp)
	global grip_res
	if(req.activate_gripper == True and resp.result == True):
	   	grip_res = 1

def dettach():
	'''
	Purpose:
	---
	This function is used to deactivate gripper and dettached box with drone

	Input Arguments:
	---
	None

	Returns:
	---
	None
	'''

	req = 0
	resp = 0
	activate = 0
	rospy.wait_for_service('/edrone/activate_gripper')
	activate = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
	req = GripperRequest(False)
	resp = activate(req)
	GripperResponse(resp)


class PositionController():

    	""" Defining class for position controller"""

    	def __init__(self):

        	rospy.init_node('position_controller')
        
    		# Initializing GPS coordinates

        	self.latitude = 0.000000
        	self.longitude = 0.000000
        	self.altitude = 0.000000

    		# Initializing Roll, Pitch, Yaw, Throttle Values

        	self.value_cmd = edrone_cmd()
        	self.value_cmd.rcRoll = 1500.0
        	self.value_cmd.rcPitch = 1500.0
        	self.value_cmd.rcYaw = 1500.0
        	self.value_cmd.rcThrottle = 1500.0
        	self.value_cmd.aux1 = 1500.0

    		# Initializing the variables for calculating errors

        	self.error = [0, 0, 0]
        	self.last_error = [0, 0, 0]
        	self.err_sum = [0, 0, 0]
        	self.d_err = [0, 0, 0]
        	self.last_time = [0, 0, 0]
        	self.now = 0
        	self.time_change = [0, 0, 0]

    		# Kp, Ki, Kd for [Latitude, Longitude, Altitude] respectively
	
		self.Kp = [200000, 200000, 900]
		self.Ki = [0, 0, 0]
		self.Kd = [1000000, 1000000, 650]

    		# Initializing variables for range sensor values

		self.front = 0
        	self.right = 0
        	self.back = 0
        	self.left = 0

    		# Initializing variables to utilize values published by marker.py

		self.err_x_m = 0
		self.err_y_m = 0
		self.alt_changer = 0
		self.x = 0
		self.y = 0
		self.box_height = 0.26
		self.flag_height = 0

    		# Accessing Delivery Locations from original.csv

		df = pd.read_csv(os.path.join(rospack.get_path('vitarana_drone'),'scripts/original.csv'), sep="[;,]",header=None,engine='python')
		pd.options.mode.chained_assignment = None 
		pd.set_option("display.precision", 10)
		df1 = df[0] == 'DELIVERY'
		df_del = df.loc[df1]
		df2 = df[0] == 'RETURN'
		df_ret = df.loc[df2]
		ret_col = list(df_ret)
		ret_col1 = ret_col
		ret_col[1], ret_col[2], ret_col[3], ret_col[4] = ret_col1[2], ret_col1[3], ret_col1[4], ret_col1[1]
		df_ret.columns = ret_col
		df_ret = df_ret.sort_index(axis = 1)
		A1 = [18.9998102845, 72.000142461, 16.757981]
		X1 = [18.9999367615, 72.000142461, 16.757981]
		lat_add = 0.000013552
		long_add = 0.000014245
		df_ret[2] = df_ret[2].astype('float64')
		df_del[4] = df_del[4].astype('float64')
		df_del[5] = df_del.index
		df_ret[5] = df_ret.index
		deli = df_del.values.tolist()
		retu = df_ret.values.tolist()
		d = 9
		sequenced = []
		pick = []
		drop = []

		# Scheduling algorithm based on the distance calculated between given locations

		while(len(retu)!=0):

			del_dist = []
			distance = math.sqrt(((A1[0] + lat_add - deli[0][2]) ** 2) + (A1[1] + long_add - deli[0][3]) ** 2)
			del_dist.append(distance)
			min_ele = 0
			for i in range(1,d):
	        		distance = math.sqrt(((A1[0] + lat_add - deli[i][2]) ** 2) + (A1[1] + long_add - deli[i][3]) ** 2)
	        		del_dist.append(distance)
	        		if(del_dist[i] > del_dist[min_ele]):
	            			min_ele = i
	    		ret_dist = []

	    		# Calculating the distance

	    		distance = math.sqrt(((deli[min_ele][2] - retu[0][2]) ** 2) + (deli[min_ele][3] - retu[0][3]) ** 2)
	    		ret_dist.append(distance)
	    		min_el = 0

	    		for i in range(1,d):

	        		distance = math.sqrt(((deli[min_ele][2] - retu[i][2]) ** 2) + (deli[min_ele][3] - retu[i][3]) ** 2)
	        		ret_dist.append(distance)

	        		if(ret_dist[i] < ret_dist[min_el]):

	            			min_el = i

	    		sequenced.append(deli[min_ele])
	    		sequenced.append(retu[min_el])
	    		del deli[min_ele]
	    		del retu[min_el]
	    		d = d - 1

		var = 0
		for var in range(0,18):

	    		if(var % 2 == 0 ):

	        		drop.append(list(sequenced[var]))
	        		lat_a = A1[0] + (ord(sequenced[var][1][0]) - 65) * 0.000013552
	        		long_a = A1[1] + (ord(sequenced[var][1][1]) - 49) * 0.000014245
	        		alt_a = 16.757981
	        		temp = [sequenced[var][0], sequenced[var][1], lat_a, long_a, alt_a, sequenced[var][5]]
	        		pick.append(list(temp))

	    		else:

	        		pick.append(list(sequenced[var]))
	        		lat_x = X1[0] + (ord(sequenced[var][1][0]) - 88) * 0.000013552
	        		long_x = X1[1] + (ord(sequenced[var][1][1]) - 49) * 0.000014245
	        		alt_x = 16.757981
	        		temp = [sequenced[var][0],sequenced[var][1],lat_x,long_x,alt_x,sequenced[var][5]]
	        		drop.append(list(temp))
	
		# Generating the output csv by appending the scheduled manifest values

		output = list(sequenced)
		for i in range(0,18):

	    		for j in range(2,5):

	        		output[i][j] = str(output[i][j])
	    			output[i][2] = ";".join(output[i][2:5])

	    		if(i % 2 == 1 ):

	        		output[i][2] = output[i][2] + "  "

		for i in range(0,18):
	    		if(i % 2 == 0):
	        		output[i] = output[i][0:3]
	    		else:
	        		output[i] = output[i][0:3]
	        		tempo = output[i][1]
	        		output[i][1], output[i][2] = output[i][2],tempo

		df_out = pd.DataFrame()
		df_out = df_out.append(output)
		df_out.to_csv(os.path.join(rospack.get_path('vitarana_drone'),'scripts/sequenced_manifest.csv'),header=None,index=None)

		self.deliver_coord = []
		for i in range(0,18):
			self.deliver_coord.append(drop[i][2:5])
		
    		# Initializing Box Locations

    		self.box_coord = []
		for i in range(0,18):
			self.box_coord.append(pick[i][2:5])

    		# Initializing some required variables

		self.marker_lat = 0
		self.marker_long = 0

    		# Initializing miscellaneous variables

		self.flag = 0
		self.new = 0
		self.flag_new = 1
		self.pubg = 0
		self.alt = 0
		self.bottom = 0
		self.new_bottom = 0

   		# Publishing the required data

        	self.pwm_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size = 1)
        	self.value_pub1 = rospy.Publisher('/altitude_pwm_out', Float32, queue_size = 1)
        	self.value_pub2 = rospy.Publisher('/lat_pwm_out', Float32, queue_size = 1)
        	self.value_pub3 = rospy.Publisher('/long_pwm_out', Float32, queue_size = 1)
		self.value_pub5 = rospy.Publisher('/new_bottom', Float32, queue_size = 1)
		self.value_var = rospy.Publisher('/y', Int8, queue_size = 1)

    		# Calling required subscribers

        	rospy.Subscriber('/edrone/gps', NavSatFix, self.edrone_gps)
        	rospy.Subscriber('/edrone/range_finder_top',LaserScan, self.range_finder)
		rospy.Subscriber('/edrone/marker_data',MarkerData, self.marker_data)
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.bottom_sense)

    	def bottom_sense(self,msg):
		'''
		Purpose:
		---
		Callback function for bottom range sensor

		Input Arguments:
		---
		msg :  [ Float32 ]
		value of bottom range sensor

		Returns:
		---
		None
		'''

		self.bottom = msg.ranges[0]

    	def marker_data(self,msg):
		'''
		Purpose:
		---
		Callback function for marker data

		Input Arguments:
		---
		msg :  [ < type of 1st input argument > ]
		   error from marker in terms of x and y coordinate 

		Returns:
		---
		None
		'''

        	self.err_x_m = msg.err_x_m
        	self.err_y_m = msg.err_y_m

    	def range_finder(self,msg):
		'''
		Purpose:
		---
		Callback function for range sensor

		Input Arguments:
		---
		msg :  [ array ]
		extracting values of all the range sensors

		Returns:
		---
		None
		'''
        	self.front = msg.ranges[0]
        	self.right = msg.ranges[1]
        	self.back = msg.ranges[2]
        	self.left = msg.ranges[3]

    	def edrone_gps(self,msg):
		'''
		Purpose:
		---
		Function to extract the values from GPS

		Input Arguments:
		---
		msg :  [ Float64 ]
		    to extract the value of latitude, longitude, altitude

		Returns:
		---
		None
		'''

        	self.latitude = msg.latitude
        	self.longitude = msg.longitude
        	self.altitude = msg.altitude

    	def set_altitude(self, get_altitude):
		'''
		Purpose:
		---
		Function set the altitude to the given input value,it Computs the PID errors for altitude,
		Computs the PWM altitude and Publishes the PWM altitude

		Input Arguments:
		---
		get_altitude :  [ float ]
		altitude to be set 

		Returns:
		---
		None
		'''

    		# Getting the current time and calculating the time change

        	self.now = float(rospy.get_time())
        	self.time_change[2] = self.now - self.last_time[2]

   		# Computing the PID errors for altitude

        	self.error[2] = get_altitude - self.altitude
        	self.err_sum[2] += self.error[2] * self.time_change[2]

        	if self.time_change[2] > 0:
            		self.d_err[2] = (self.error[2]-self.last_error[2]) / self.time_change[2]

        	self.last_time[2] = self.now
        	self.last_error[2] = self.error[2]

    		# Computing the PWM altitude (output)

        	self.pwm_altitude = self.Kp[2] * self.error[2] + self.Ki[2] * self.err_sum[2] + self.Kd[2] * self.d_err[2]

    		# Publishing the PWM altitude

        	self.value_pub1.publish(self.pwm_altitude)
        	self.pwm_pub.publish(self.value_cmd)

    	def set_latitude(self, get_latitude):
	    	'''
		Purpose:
		---
		Function set the latitude to the given input value,
		it Computs the PID errors for latitude,
		Computs the PWM latitude and Publishes the PWM latitude

		Input Arguments:
		---
		get_latitude :  [ float ]
		latitude to be set 

		Returns:
		---
		None
		'''

    		# Getting the current time and calculating the time change

            	self.now = float(rospy.get_time())
            	self.time_change[0] = self.now - self.last_time[0]

    		# Computing the PID errors for latitude

            	self.error[0] = get_latitude - self.latitude
            	self.err_sum[0] += self.error[0] * self.time_change[0]

            	if self.time_change[0] > 0:
                	self.d_err[0] = (self.error[0]-self.last_error[0]) / self.time_change[0]

            	self.last_time[0] = self.now
            	self.last_error[0] = self.error[0]

    		# Computing the PWM latitude (output)

            	self.pwm_latitude = self.Kp[0] * self.error[0] + self.Ki[0] * self.err_sum[0] + self.Kd[0] * self.d_err[0]

    		# Publishing the PWM latitude

            	self.value_pub2.publish(self.pwm_latitude)
            	self.pwm_pub.publish(self.value_cmd)

    	def set_longitude(self, get_longitude):
		'''
		Purpose:
		---
		Function set the longitude to the given input value,
		it Computs the PID errors for longitude,
		Computs the PWM latitude and Publishes the PWM longitude

		Input Arguments:
		---
		get_latitude :  [ float ]
		latitude to be set 

		Returns:
		---
		None
		'''
    		# Getting the current time and calculating the time change

            	self.now = float(rospy.get_time())
            	self.time_change[1] = self.now - self.last_time[1]

    		# Computing the PID errors for altitude

            	self.error[1] = get_longitude - self.longitude
            	self.err_sum[1] += self.error[1] * self.time_change[1]

            	if self.time_change[1] > 0:
                	self.d_err[1] = (self.error[1]-self.last_error[1]) / self.time_change[1]

            	self.last_time[1] = self.now
            	self.last_error[1] = self.error[1]

    		# Computing the PWM altitude (output)

            	self.pwm_longitude = self.Kp[1] * self.error[1] + self.Ki[1] * self.err_sum[1] + self.Kd[1] * self.d_err[1]
            
    		# Publishing the PWM latitude

            	self.value_pub3.publish(self.pwm_longitude)
            	self.pwm_pub.publish(self.value_cmd)

    	def follow_wall_go_front(self, lat):
		'''
		Purpose:
		---
		Defining functions to follow the obstacles 

		Input Arguments:
		---
		lat :  [ float ]
		latitude coordinates to maintain distance con

		Returns:
		---
		None
		'''

	    	self.set_latitude(lat)
	    	self.set_longitude(self.longitude + 0.0001)

    	def follow_wall_go_back(self, lat):
		'''
		Purpose:
		---
		Defining functions to follow the  front obstacles 

		Input Arguments:
		---
		lat :  [ Float32 ]
		value of front range sensor in terms of latitude

		Returns:
		---
		None
		'''

	    	self.set_latitude(lat)
	    	self.set_longitude(self.longitude - 0.0001)

    	def follow_wall_go_left(self,longi):
		'''
		Purpose:
		---
		Defining functions to follow the left obstacles 

		Input Arguments:
		---
		lat :  [ Float32 ]
		value of front range sensor in terms of longitude

		Returns:
		---
		None
		'''

	    	self.set_latitude(self.latitude + 0.0001)
	    	self.set_longitude(longi)

    	def follow_wall_go_right(self,longi):
		    '''
		Purpose:
		---
		Defining functions to follow the right obstacles 

		Input Arguments:
		---
		lat :  [ Float32 ]
		value of front range sensor in terms of longitude

		Returns:
		---
		None
		'''

	    	self.set_latitude(self.latitude - 0.0001)
	    	self.set_longitude(longi)

    	def go_to_pos(self , lati , longi):
		'''
		Purpose:
		---
		Function for sending the drone to required position

		Input Arguments:
		---
		lati :  [ Float32 ]
		latitude where the drone has to go

		longi :  [ Float32 ]
		longitude where the drone has to go

		Returns:
		---
		None

		Example call:
		---
		'''

	    	self.set_latitude(lati)
	    	self.set_longitude(longi)
    		
    	def arm(self):
		'''
		Purpose:
		---
		Defining function for arming the drone

		Input Arguments:
		---
		None

		Returns:
		---
		None
		'''

        	self.value_cmd.aux1 = 1500.0

    	def disarm(self):
		'''
		Purpose:
		---
		Defining function for disarming the drone  

		Input Arguments:
		---
		None

		Returns:
		---
		None
		'''

        	self.value_cmd.aux1 = 1000.0

    	def lat_to_x(self, input_latitude):
		'''
		Purpose:
		---
		conversion of latitude to x(in meters)

		Input Arguments:
		---
		input_latitude :  [ float ]
		latitude value

		Returns:
		---
		directly return in meters :  [ float ]
		convert the value in meters and return it
		'''

        	return 110692.0702932625 * (input_latitude - 19)

    	def long_to_y(self, input_longitude):
		'''
		Purpose:
		---
		conversion of longitude to x(in meters)

		Input Arguments:
		---
		input_longitude :  [ float ]
		longitude value

		Returns:
		---
		directly return in meters :  [ float ]
		convert the value in meters and return it
		'''

        	return -105292.0089353767 * (input_longitude - 72)

    	def x_to_lat(self, input_x):
		'''
		Purpose:
		---
		conversion of x(in meters) to latitude

		Input Arguments:
		---
		input_x :  [ float ]
		x in meters

		Returns:
		---
		directly return latitude :  [ float ]
		convert the value in latitude and return it
		'''

        	return ((input_x / 110692.0702932625 ) + 19)

    	def y_to_long(self, input_y):
		'''
		Purpose:
		---
		conversion of y(in meters) to longitude

		Input Arguments:
		---
		input_y :  [ float ]
		y in meters

		Returns:
		---
		directly return latitude :  [ float ]
		convert the value in longitude and return it
		'''

        	return ((input_y / ( -105292.0089353767 )) + 72)

    
    	def new_bottom_func(self,var):
		'''
		Purpose:
		---
		Defining a function to calculate the bottom range sensor value

		Input Arguments:
		---
		var :  [ int ]
		Index of deliver_coord to get the altitude of delivery

		Returns:
		---
		None
		'''

		self.new_bottom = self.altitude - self.deliver_coord[var][2] 
		self.value_pub5.publish(self.new_bottom)


    	def pos_check(self, m, n, t, u = 6):
		'''
		Purpose:
		---
		Defining a function to check whether the drone has reached to the given location or not

		Input Arguments:
		---
		m :  [ float ]
		latitude location that has to be achieved

		n :  [ float ]
		longitude location that has to be achieved

		t :  [ int ]
		decide the direction of position setting

		u :  [ int ]
		the value till which we need to truncate

		Returns:
		---
		return bool value :  [ bool ]
		return the bool value if the drone reached or not
		'''

		if(t == 1 and self.truncate(self.latitude,u) <= self.truncate(m,u) and self.truncate(self.longitude,u) <= self.truncate(n,u)):
			return True
		elif(t == 2 and self.truncate(self.latitude,u) >= self.truncate(m,u) and self.truncate(self.longitude,u) >= self.truncate(n,u)):
		    	return True
		elif(t == 3 and self.truncate(self.latitude,u) <= self.truncate(m,u) and self.truncate(self.longitude,u) >= self.truncate(n,u)):
		    	return True
		elif(t == 4 and self.truncate(self.latitude,u) >= self.truncate(m,u) and self.truncate(self.longitude,u) <= self.truncate(n,u)):
		    	return True
		else:
		    	return False

    	def truncate(self,f, n):
		'''
		Purpose:
		---
		Function to truncate values upto certain decimal places

		Input Arguments:
		---
		f :  [ Float32 ]
		Value to be truncted

		n :  [ Int8 ]
		upto which number value to be truncated

		Returns:
		---
		directly return the value :  [ Float32 ]
		truncated value
		'''

    		return math.floor(f * 10 ** n) / 10 ** n

    	def pick_box(self, var_1):
		'''
		Purpose:
		---
		Function for picking up the box from the given location

		Input Arguments:
		---
		var_1 :  [ int ]
		Index of box_coord to get the latitude and longitude of box position

		Returns:
		---
		None
		'''

		self.Kp = [200000, 200000, 900]
		self.Kd = [1000000, 1000000, 650]

		if(self.alt_changer == 0):

			prev_height = self.box_coord[var_1-1][2]
			curr_height = self.box_coord[var_1][2]

			if(prev_height > curr_height):
				self.set_altitude(prev_height + 5)
			else:
				self.set_altitude(curr_height + 6)

			# Conditon to check whether the lat and long is increasing or decreasing

			if(self.box_coord[var_1][0] <= self.latitude and self.box_coord[var_1][1] <= self.longitude):
				self.pubg = 1
			elif(self.box_coord[var_1][0] >= self.latitude and self.box_coord[var_1][1] >= self.longitude):
				self.pubg = 2
			elif(self.box_coord[var_1][0] <= self.latitude and self.box_coord[var_1][1] >= self.longitude):
				self.pubg = 3
			else:
				if(self.box_coord[var_1][0] >= self.latitude and self.box_coord[var_1][1] <= self.longitude):
					self.pubg = 4

		self.flag_new = 1

		if(self.flag_new == 1):

			# Setting the lat, long of the pick up location

			self.go_to_pos(self.box_coord[var_1][0], self.box_coord[var_1][1])

			if(self.pos_check(self.box_coord[var_1][0],self.box_coord[var_1][1],self.pubg)):

				self.alt_changer = 1
				self.y += 1

		if(self.alt_changer == 1):

			self.set_altitude(self.box_coord[var_1][2])

	

    	def drop_box(self, var_2):
		'''
		Purpose:
		---
		Function to go the delivery coordinates

		Input Arguments:
		---
		var_2 :  [ int ]
		Index of deliver_coord to get the latitude and longitude of delivery

		Returns:
		---
		None
		'''

		self.alt = 13

		if(var_2 % 2 == 0):

			if(self.alt_changer == 1):

				if(self.flag_height == 0):

					self.set_altitude(self.deliver_coord[var_2][2] + self.alt)

				# Conditon to check whether the lat and long is increasing or decreasing

				if(self.deliver_coord[var_2][0] <= self.latitude and self.deliver_coord[var_2][1] <= self.longitude):
					self.pubg = 1
				elif(self.deliver_coord[var_2][0] >= self.latitude and self.deliver_coord[var_2][1] >= self.longitude):
					self.pubg = 2
				elif(self.deliver_coord[var_2][0] <= self.latitude and self.deliver_coord[var_2][1] >= self.longitude):
					self.pubg = 3
				else:
					if(self.deliver_coord[var_2][0] >= self.latitude and self.deliver_coord[var_2][1] <= self.longitude):
						self.pubg = 4

				self.flag = 1

			if(self.flag == 1):

				# Reaching to the delivery coordinate by setting the respective lat and long

				self.Kp = [69000, 69000, 900]
				self.Kd = [605000, 605000, 650]

				self.go_to_pos(self.deliver_coord[var_2][0], self.deliver_coord[var_2][1])
				if(self.pos_check(self.deliver_coord[var_2][0], pos.deliver_coord[var_2][1], self.pubg, 4)):
					self.flag_height= 1
					self.set_altitude(self.deliver_coord[var_2][2] + self.alt - 4)
					if(self.altitude <= self.deliver_coord[var_2][2] + self.alt - 3.5):
						self.alt_changer = 2

			if(self.alt_changer == 2 ):

				self.flag = 2

			if(self.flag == 2):
				if(self.alt_changer ==2):
					self.set_altitude(self.deliver_coord[var_2][2] + self.alt - 4)

				self.Kp = [200000, 200000, 900]
				self.Kd = [1000000, 1000000, 650]

				if(self.new == 0):

					self.marker_lat = self.lat_to_x(self.latitude) + self.err_x_m
					self.marker_long = self.long_to_y(self.longitude) + self.err_y_m
					self.new = 1

				self.go_to_pos(self.x_to_lat(self.marker_lat), self.y_to_long(self.marker_long))
				if(abs(self.err_x_m) <= 0.3 and abs(self.err_y_m) <= 0.3):
					self.alt_changer = 3

			if(self.alt_changer == 3):
				self.set_altitude(self.deliver_coord[var_2][2])
				if(self.altitude <= self.deliver_coord[var_2][2] + self.box_height+1):
					self.alt_changer = 0
					self.flag = 0
					self.new = 0
					self.x += 1
					self.flag_height = 0

		# To bring the box back to the warehouse

		else:
			if(self.alt_changer == 1):

				self.set_altitude(self.deliver_coord[var_2][2] + 10)

				# Conditon to check whether the lat and long is increasing or decreasing

				if(self.deliver_coord[var_2][0] <= self.latitude and self.deliver_coord[var_2][1] <= self.longitude):
					self.pubg = 1
				elif(self.deliver_coord[var_2][0] >= self.latitude and self.deliver_coord[var_2][1] >= self.longitude):
					self.pubg = 2
				elif(self.deliver_coord[var_2][0] <= self.latitude and self.deliver_coord[var_2][1] >= self.longitude):
					self.pubg = 3
				else:
					if(self.deliver_coord[var_2][0] >= self.latitude and self.deliver_coord[var_2][1] <= self.longitude):
						self.pubg = 4

				self.flag = 1

			if(self.flag == 1):

				# Reaching to the delivery coordinate by setting the respective lat and long

				self.Kp = [69000, 69000, 900]
				self.Kd = [605000, 605000, 650]

				self.go_to_pos(self.deliver_coord[var_2][0], self.deliver_coord[var_2][1])
				if(self.pos_check(self.deliver_coord[var_2][0], self.deliver_coord[var_2][1], self.pubg)):
					self.alt_changer = 2

			if(self.alt_changer == 2 and self.altitude >= self.deliver_coord[var_2][2] + 10):

				self.flag = 2
				self.alt_changer = 3


			if(self.alt_changer == 3):
				self.set_altitude(self.deliver_coord[var_2][2] + 0.1)

				if(self.altitude <= self.deliver_coord[var_2][2] + self.box_height ):
					self.alt_changer = 0
					self.flag = 0
					self.new = 0
					self.x += 1

if __name__ == '__main__':
	
    	# Creating an instance of the class

    	pos = PositionController()
    	r = rospy.Rate(50)

    	# Loop for running the rospy
	
    	while not rospy.is_shutdown():

		# Getting the initial coordinates

		if(init_flag == 0):

			init_lat = pos.latitude
			init_long = pos.longitude
			init_alt = pos.altitude

			if(init_lat != 0.0 and init_long != 0.0 and init_alt != 0.0):
				init_flag = 1

		# Condition for attaching the box

		pos.new_bottom_func(coord_y)

		# Activating gripper to attach the box

		if(grip_res == 0):
			attach()
			pos.set_altitude(pos.box_coord[coord_x][2] - 1)

		# Checking whether box is grabbed

		if(grip_res == 1):

			# Condition for taking the box to building

			pos.drop_box(coord_y)

			if(pos.alt_changer == 0):
				if(pos.altitude <= pos.deliver_coord[coord_y][2] + pos.box_height + 1):
					grip_res = 3
					coord_x = pos.x

		# Condition to detach the box at the detected marker and return to warehouse over the next box

		if(grip_res == 3):
			dettach()

			# Condition for arriving at first box by setting the lat,long and alt accordingly

			if(coord_x == 0):
				if(pos.alt_changer == 0):
					pos.set_altitude(pos.box_coord[0][2] + 1)

				if(pos.altitude >= init_alt + 1 and pos.flag == 0):
					pos.go_to_pos(pos.box_coord[0][0], pos.box_coord[0][1])

					if(pos.truncate(pos.latitude, 6) <= pos.truncate(pos.box_coord[0][0], 6) and pos.truncate(pos.longitude, 6) >= pos.truncate(pos.box_coord[0][1], 6)):
						pos.alt_changer = 1

				if(pos.alt_changer == 1):
					grip_res = 0

			# Condition for detaching the box and returning to warehouse 

			else:
				pos.pick_box(coord_x)
				if(pos.alt_changer == 1):
					grip_res = 0
					coord_y = pos.y



			# Condition for detaching the last box and returning to warehouse at initial position by setting the lat,long and alt accordingly

			if(coord_x == 18):
				if(pos.alt_changer == 0):
					pos.set_altitude(pos.deliver_coord[17][2] + 2)

				pos.flag_new = 1

				if(pos.truncate(pos.latitude, 6) <= pos.truncate(init_lat, 6) and pos.truncate(pos.longitude, 6) <= pos.truncate(init_long, 6)):
					pos.alt_changer = 1

				if(pos.alt_changer == 1):
					pos.set_altitude(pos.truncate(init_alt, 2))
					pos.disarm()

        	r.sleep()
	