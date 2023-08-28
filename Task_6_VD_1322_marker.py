'''
# Team ID:          VD_#1322
# Theme:            Vitarana Drone
# Author List:      Prakhar Agrawal, Abha Porwal, Apoorv Nema, Kanak Verma
# Filename:         Task_6_VD_1322_marker
# Functions:        image_callback, new_bottom, building_no
# Global variables: focal_length, x_cen, y_cen
'''
# Importing necessary libraries

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from vitarana_drone.msg import *
import cv2
from matplotlib import pyplot as plt
import rospy, rospkg
import numpy as np
from std_msgs.msg import Float32, Int8
import time
import os
import math

rospack = rospkg.RosPack()
rospack.list()

# Initializing some global variables

focal_length = 0
x_cen = 0
y_cen = 0

# Defining class for processing marker images


class image_proc():

	# Initialise everything

        def __init__(self):
                '''
                Purpose:
                ---
                Function for calling all the publisher ans subscribers
                and to initialize all the parameters.
                Input Arguments:
                ---
                None
                Returns:
                ---
                None
                '''
                self.bottom = 0
                self.value_cmd = MarkerData()
                self.value_cmd.err_x_m = 0
                self.value_cmd.err_y_m = 0
                # Initialise rosnode
                rospy.init_node('marker')
                # Subscribing to the camera topic and range bottom sensor 
                self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 
                rospy.Subscriber('/new_bottom', Float32, self.new_bottom)
                self.value_pub = rospy.Publisher('/edrone/marker_data', MarkerData, queue_size = 1)
                self.img = np.empty([])  # This will contain your image im from camera
                self.bridge = CvBridge()

        def image_callback(self, data):
                '''
                Purpose:
                ---
                Callback function of camera topic
                Input Arguments:
                ---
                self  :  [ class  ]
                data  :  [  matrix  ]
                It is the image obtained by the camera of the drone
                Returns:
                ---
                None
                '''
                try:
                	# Converting the image to OpenCV standard image
                	self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

                except CvBridgeError as e:
                	print(e)
                	return

        def new_bottom(self, msg):
                '''
                Purpose:
                ---
                Function to extract values from range finder bottom sensor
                Input Arguments:
                ---
                msg : [Int8]
                It is the value of range bottom sensor
                Returns:
                ---
                None
                '''
                self.bottom = msg.data

if __name__ == '__main__':
	
        image_proc_obj = image_proc()
        r = rospy.Rate(50)        
        # Accessing the webcam
        
        cap = cv2.VideoCapture(0)
        
        # Calculating focal length required
        
        focal_length = (400 / 2) / math.tan(1.3962634 / 2)
        while not rospy.is_shutdown():

                # Creating the object

                img = image_proc_obj.img

        # Getting the weights calculated by training the images

                logo_cascade = cv2.CascadeClassifier(os.path.join(rospack.get_path
                	('vitarana_drone'), 'scripts/data/cascade.xml'))

        # Coverting the RGB to grey image

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # image, reject levels level weights.

        logo = logo_cascade.detectMultiScale(gray, scaleFactor = 1.05)

    	# Getting the coordinates of the frame

        for (x, y, w, h) in logo:
	
    		# Drawing rectangle around the marker
	
    		cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)

    		# Printing the respective position of marker on the image

    		txt = str(x + w // 2) + str(" , ") + str(y + h // 2) 

    		# Calculating the image centre

    		x_cen = -(200 - (x + w / 2)) * image_proc_obj.bottom / focal_length
    		y_cen = (200 - (y + h / 2)) * image_proc_obj.bottom / focal_length

    		# Command to put some text on the image

    		cv2.putText(img, txt, (x - 10, y - 10), 
		    	        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

        	cv2.imshow("DETECT MARKER", img)
	
	# Publishing some values
	
        image_proc_obj.value_cmd.err_x_m = x_cen
	image_proc_obj.value_cmd.err_y_m = y_cen
        image_proc_obj.value_pub.publish(image_proc_obj.value_cmd)
        key = cv2.waitKey(1)
        r.sleep()
