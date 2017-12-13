#!/usr/bin/env python
"""
Author: Samir Wadhwania, Martina Stadler, Jose Gomez, Clementine Mitchell
This program recieves an image from a rostopic and converts it
to a cv2 image.  Then, it performs a CV algorithm to detect a cone
in the image. The distance of the cone and the shift from the center is
passed to the controller.
"""
import cv2  # the open cv library packge
import rospy # standard ros with python package
import numpy as np
import copy
import matplotlib as plt

import math
import csv
import scipy.linalg
from matplotlib import colors
from sensor_msgs.msg import Image  # the rostopic message we subscribe/publish 
from cv_bridge import CvBridge # package to convert rosmsg<->cv2 
from std_msgs.msg import Float32MultiArray, Float32

class Pursuit_Detector:
    def __init__(self):
        
        # create bridge to convert to and from cv2 and rosmsg
        self.bridge = CvBridge()

        self.left_contour_y = None
        self.left_contour_x  = None
        self.width = None
        self.ft_to_m = 0.3048
        
        ######## CLEMMIE - NEED TO CHECK ###########
        # Need to work out where to put these things such that they reset after 
        # every 5 samples are added (before a new cone is found) but not every time
        # we get a new image?
        self.worldx_samples = np.array([])
        self.worldy_samples = np.array([])
        # The number of values we have already sampled to take an average of the cone's position
        self.sampled_vals_count = 0
        
        # We will sample values within 0.2 feet of 8ft, 7ft, 6ft,... for the correct number of samples
        self.depth_tolerance = 0.2
        
        # The number of samples we want to average over.
        self.number_of_samples = 5        
        ##############################

        degree = 3
        ycam = np.array([278, 230, 211, 197, 190])
        yworld = np.array([2, 3, 4, 5, 6])
        
        self.polyfityworld = np.polyfit(ycam, yworld, degree)

        xfootspacing = np.array([536-296, 469-322, 438-336, 421-342, 411-347, 404-349])
        yspacing = np.array([2, 3, 4, 5, 6, 7])

        self.polyfitfootratio = np.polyfit(yspacing, xfootspacing, degree)

        self.coeffsX, self.coeffsY = self.read_data()
        
        # subscribe to the rostopic carrying the image we are interested in
        # "camera/rgb/image_rect_color" is the topic name
        # Image is the message type
        # self.processImage is the callback function executed every time we
        # receive the message
        
        self.sub_image = rospy.Subscriber("/zed/rgb/image_rect_color",\
                Image, self.processImage, queue_size=1)
         
        # publisher for the control inputs we will output
        self.pub_control = rospy.Publisher("pursuit_control_input",\
                Float32MultiArray, queue_size=1)

        self.pub_image = rospy.Publisher("cv_image",\
                Image, queue_size=1)

        # report initalization success
        rospy.loginfo("CV Node Initialized.")

    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic we subscribed to.
    """
    def processImage(self, image_msg):
        # cone color variable
        cone_color = None
        # Convert the image from BGR to HSV
        image_msg = self.bridge.imgmsg_to_cv2(image_msg)
        height, width, channels = image_msg.shape
           

        # Convert the image from BGR to HSV
        hsv = cv2.cvtColor(image_msg, cv2.COLOR_BGR2HSV)

        sliced_height = int(round(height*0.25))

        image_cut = hsv[sliced_height:]


        #print image_msg
        #hsv = cv2.cvtColor(image_msg, cv2.COLOR_BGR2HSV)

        # This sets the limits for what is considered "orange". A pixel with values in the range lower-upper for H, S, and V is considered part of the cone.
        # These limts are currently HARDCODED for an orange cone. Really bright lighting, etc, could mess this up.
        # The limits are in HSV. In cv2, hue ranges fr2424e2a014f5ccafc18b1121c493387bba9f878fom 0-179, and saturation ranges from 0-255.
        upper_orange = np.array([16, 255, 255])

        # Lower orange bound for photos from our robot
        lower_orange = np.array([6, 200, 135])

        # Lower orange bound for stock footage
        # lower_orange = np.array([1, 20, 20])

        # Initialized to orange values
        upper_green = np.array([90, 255, 255])

        lower_green = np.array([70, 40, 35])

        try:
            # Apply upper and lower limits, where pixels inside limits -> white and pixels outside of limits -> black.
            mask_orange = cv2.inRange(image_cut, lower_orange, upper_orange)
            mask_green = cv2.inRange(image_cut, lower_green, upper_green)

            mask = cv2.bitwise_or(mask_orange, mask_green)

            image_orange, contours_orange, hierarchy_orange = cv2.findContours(mask_orange, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            image_green, contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        
            try:
                longest_contour_orange = np.array(max(contours_orange, key=len))
            except:
                longest_contour_orange = np.array([])
            try:
                longest_contour_green = np.array(max(contours_green, key=len))
            except:
                longest_contour_green = np.array([])

            # If statements to set longest contours and color 1 for green and 0 for orange.
            if len(longest_contour_orange) == 0:
                cone_color = 1
                longest_contour = np.array(longest_contour_green)
            elif len(longest_contour_green) == 0:
                longest_contour = np.array(longest_contour_orange)
                cone_color = 0
            elif len(longest_contour_orange) > len(longest_contour_green):
                longest_contour = np.array(longest_contour_orange)
                cone_color = 0
            else:
                longest_contour = np.array(longest_contour_green)
                cone_color = 1

            # Find moments corresponding to all contour points
            M = cv2.moments(np.array(longest_contour))
            # Find centroid of cone
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            #print cx, cy

            # Flattening all contour points list
            flat_contour_pts = longest_contour.flat

            # Filtering out X coordinates
            contour_x_coord = flat_contour_pts[0:len(longest_contour.flat):2]

            # Finding max and min x-coordinates (edge points)
            min_x = min(contour_x_coord)
            max_x = max(contour_x_coord)

            index_min = np.where(contour_x_coord==min_x)
            index_max = np.where(contour_x_coord==max_x)

            min_y = flat_contour_pts[index_min[0][0]*2+1]
            max_y = flat_contour_pts[index_max[0][0]*2+1]

            # avg y coordinate
            avg_y = np.mean([min_y, max_y])


            # Find a bounding box for the cone.
            x, y, w, h = cv2.boundingRect(np.array(longest_contour))

            #print 'x, y, w, h', x, y, w, h
            bbox = cv2.rectangle(image_msg, (x, y+sliced_height), (x+w, y+h+sliced_height), (0, 255, 0), 1)

            centroid_img = cv2.circle(bbox, (cx, cy+sliced_height), 5, (255, 0, 0), -1)
            #cv2.imwrite("Processed_"+image_msg_file_string,centroid_img)
            #cv2.imshow('bb cont', centroid_img)

            # Calculate shift
            shift = float(cx)/width - 0.5
            scaled_cx = x/float(width)
            # scaled_cy = (cy+sliced_height)/float(height)
            scaled_cy = float(y+h+sliced_height)/float(height)
            # scaled_cx = x/float(width)

            worldx, worldy = self.coord_transform2(scaled_cx, scaled_cy)
            if worldy<20:
                worldy += np.polyval([0.0043, -0.0462, 0.1247, -0.2531, 0.0032],worldy)
            # Still need to fix worldx
            # worldx = np.sin((scaled_cx*2-1))*worldy
                

            # if worldy>6.9:
            #     worldy += (2.43/1.57)*(worldy - 6.93) - 0.93
            #     if worldy>7.5 and worldy<9.6:
            #         worldy -= 0.6
            # else:
            #     worldy -= (0.63/5.63)*(worldy - 1.3) + 0.3


            ####### CLEMMIE - Add a sample at 8ft, 7ft etc to the values to average over ####
            # And publish when we have a pre-determined number of samples
            
            if abs(worldy-(8-self.sampled_vals_count)) < self.depth_tolerance:
                self.worldy_samples.append(worldy)
                self.worldx_samples.append(worldx)
                self.sampled_vals_count += 1

            if len(self.worldy_samples) == self.number_of_samples:
                publish_message(self, centroid_img, cone_color)

            #########
    
            print worldx,worldy,cone_color
            # line_fit = np.polyfit(worldx,worldy,3)
            # val = np.polyval(line_fit, worldx)
            
        except:
            print len(longest_contour_orange), len(longest_contour_green)


    #### CLEMMIE - Only publish when we have a set number of samples and we can average ######
    def publish_message(self, centroid_img, cone_color):
            # Publishing cone coordinates and color [x,y,color]
            worldx_ave = np.mean(self.worldx_samples)  
            worldy_ave = np.mean(self.worldy_samples)
    
            controlMsg = Float32MultiArray()
            controlMsg.data = [worldx_ave*self.ft_to_m, worldy_ave*self.ft_to_m, cone_color]
            self.pub_control.publish(controlMsg)
            self.worldx_samples = np.array([])
            self.worldy_samples = np.array([])
            self.sampled_vals_count = 0

            # convert cv2 message back to rosmsg
            image_ros_msg = self.bridge.cv2_to_imgmsg(centroid_img, "bgr8")

            # publish rosmsg 
            self.pub_image.publish(image_ros_msg)
            
    ########
        
    
    def coord_transform(self):

        flipped_y = np.array(self.left_contour_y)*(0.525)
        scaledx = np.array(self.left_contour_x)*(0.522)


        y_world = np.polyval(self.polyfityworld, flipped_y)
        # print y_world
        ratio = np.polyval(self.polyfitfootratio, y_world)
        left_contour_x = np.array(self.left_contour_x) - float(self.width)/2.0
        x_world = np.divide(left_contour_x, ratio)

        plt.plot(x_world,y_world)
        plt.show()
    
    def read_data(self):
        # reading may be wrong, try implementing as in trajectory.py with the car, then debugg this problem out
        reader = csv.reader(open('/home/racecar/racecar-ws/src/lab4/src/dataset_straight.csv', 'rU'), dialect = csv.excel_tab)

        data_points = []
        row_count = 1
        for row in reader:
          if row_count == 1:
              pass
          else:
              data_points.append(row)
          row_count += 1

        corrected_data = []
        for data_point in data_points:
          new_point = []
          for point in data_point[0].split(','):
              new_point.append(float(point))
          corrected_data.append(new_point)

        np_data_array = np.array(corrected_data)

        u = np_data_array[:, 0]
        v = np_data_array[:, 1]
        x = np_data_array[:, 2]
        y = np_data_array[:, 3]


        data = np.c_[u,v,x]

        # regular grid covering the domain of the data
        mn = np.min(data, axis=0)
        mx = np.max(data, axis=0)
        X,Y = np.meshgrid(np.linspace(mn[0], mx[0], 20), np.linspace(mn[1], mx[1], 20))
        XX = X.flatten()
        YY = Y.flatten()

        # best-fit quadratic curve
        A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
        C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])

        # Converty to data
        data = np.c_[u,v,y]

        # regular grid covering the domain of the data
        mn = np.min(data, axis=0)
        mx = np.max(data, axis=0)
        X,Y = np.meshgrid(np.linspace(mn[0], mx[0], 20), np.linspace(mn[1], mx[1], 20))
        XX = X.flatten()
        YY = Y.flatten()

        # best-fit quadratic curve
        B = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
        D,_,_,_ = scipy.linalg.lstsq(B, data[:,2])

        return C, D

    def coord_transform2(self, u, v):
        x = self.coeffsX[4]*np.multiply(u,u) + self.coeffsX[5]*np.multiply(v,v) + self.coeffsX[3]*np.multiply(u,v) + self.coeffsX[1]*u + self.coeffsX[2]*v + self.coeffsX[0]
        y = self.coeffsY[4]*np.multiply(u,u) + self.coeffsY[5]*np.multiply(v,v) + self.coeffsY[3]*np.multiply(u,v) + self.coeffsY[1]*u + self.coeffsY[2]*v + self.coeffsY[0]
        return (x, y)





if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('Cone_Detector')

    # create Echo to start the image passthrough
    c = Pursuit_Detector()
    # pub_control = rospy.Publisher("pursuit_control_input",Float32MultiArray, queue_size=1)
    # controlMsg = Float32MultiArray()
    # controlMsg.data = [0.0, 2.0, 1]
    # pub_control.publish(controlMsg)



    # continue running echo until node is killed from outside process
    rospy.spin()
