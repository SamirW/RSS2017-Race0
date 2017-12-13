#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np

"""
Author: Ariel Anders
This program implements a simple safety node based on laser
scan data.

Edited by: Winter Guerra

# Single scan from a planar laser range-finder
# This is the ROS message structure

Header header
# stamp: The acquisition time of the first ray in the scan.
# frame_id: The laser is assumed to spin around the positive Z axis
# (counterclockwise, if Z is up) with the zero angle forward along the x axis

float32 angle_min # start angle of the scan [rad]
float32 angle_max # end angle of the scan [rad]
float32 angle_increment # angular distance between measurements [rad]

float32 time_increment # time between measurements [seconds] - if your scanner
# is moving, this will be used in interpolating position of 3d points
float32 scan_time # time between scans [seconds]

float32 range_min # minimum range value [m]
float32 range_max # maximum range value [m]

float32[] ranges # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities # intensity data [device-specific units]. If your
# device does not provide intensities, please leave the array empty.

"""

class Safety():
	def __init__(self):

		# Init subscribers and publishers
		self.sub = rospy.Subscriber("/scan", LaserScan,\
				self.safetyCB, queue_size=1)
				
		self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety",\
				AckermannDriveStamped, queue_size =1 )
				
		rospy.loginfo("Safety node initialized")


	def safetyCB(self, msg):
		'''
		This callback is called everytime the laserscanner sends us data.
		This is about 40hz. The message received is described above.  The 
		240 range points received are used to determine if there is an 
		object in front of the racecar (at its heading).  If there is, 
		the car will back up and stop.
		'''

		try:
			backing_up

		except NameError:
			backing_up = False


		if not backing_up:

			something_in_front_of_robot_left = False 
			something_in_front_of_robot_right = False 
			
			# Can change filter value for accuracy etc.
			low_pass_filter = 0.35
		
			# msg_cut will be the range data with values above and below the thresholds discarded.
			msg_cut = list(msg.ranges[360:601])
		
			# Removing values not in the range of the laser scanner
			# Maximum range is approximately 30m and minimum range is approximately 0.1m (found in msg)
			for i in range(360,601):
					if msg.ranges[i] > msg.range_max or msg.ranges[i] < msg.range_min:
						msg_cut.remove(msg.ranges[i])
		
			# Smooth data (remove noise) with a low-pass filter
			for j in range(1,len(msg_cut)): 
				msg_cut[j] = low_pass_filter * msg_cut[j] + (1 - low_pass_filter) * msg_cut[j-1]

			# Split data into left and right side obstacles
			left_front = msg_cut[0:121]
			right_front = msg_cut[121:241]

			# Say that there is something in front of the robot if the minimum value in the filtered data is less than 0.2m.
			# Check both left and right side. Result will determine if we steer as we back up, and which way to steer
			if min(left_front) < 0.2:
				something_in_front_of_robot_left = True

			if min(right_front) < 0.2:
				something_in_front_of_robot_right = True
			
				# If the autonomous stack is trying to run us into a wall, back up, turn if necessary
				if ( something_in_front_of_robot_left or something_in_front_of_robot_right):
					drive_msg = AckermannDriveStamped()
					drive_msg.drive.speed = -0.1

				if something_in_front_of_robot_left:
					drive_msg.drive.steering_angle = -0.1

				elif something_in_front_of_robot_right:
					drive_msg.drive.steering_angle = 0.1

				if something_in_front_of_robot_left and something_in_front_of_robot_right: 
					drive_msg.drive.steering_angle = 0.0

				# Record time of starting back up
				backing_up = True
				start_backup_time = drive_msg.header.stamp.secs + drive_msg.header.stamp.nsecs/(10**9)

				self.pub.publish(drive_msg)

		else:
			# Stop backing up after 1.0 seconds
			drive_msg = AckermannDriveStamped()
			sim_time = drive_msg.header.stamp.secs + drive_msg.header.stamp.nsecs/(10**9)
			if abs(sim_time - start_backup_time) >= 1.0:
				backing_up = False

if __name__=="__main__":
	# Tell ROS that we're making a new node.
	rospy.init_node("Safety_Node")

	# Init the node
	Safety()

	# Don't let this script exit while ROS is still running
	rospy.spin()
