#!/usr/bin/env python
import rospy
import numpy as np
import math

from std_msgs.msg import String, Header, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped


class PurePursuit():
	def __init__(self):
		'''
		Init the node
		'''

		### Init the subscribers ###

		# Subscribe to path
		self.subPath = rospy.Subscriber(rospy.get_param("~path_topic", "/path"), Float32MultiArray, \
			self.setPath, queue_size=1)

		# Subscribe to current pose
		self.subPose = rospy.Subscriber("/infered_pose_map", PoseStamped, self.updateControl, \
			queue_size=1)

		### Init the publishers ###

		# Output driving control
		self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
				AckermannDriveStamped, queue_size =1)

		rospy.loginfo("Pure pursuit node initialized")

		# Set parameters
		self.steering_gain = 75
		self.speed = 2.0
		self.Lfw = self.steering_gain * self.speed	# Look ahead distance
		self.pathSet = False
		self.path = None
		self.pathSegs = None
		self.pathSegLenSq = None
		self.position = None
		self.distWheels = 1.05 # feet

		def setPath(self, message):
			pathList = message.data
			self.path = np.array(zip(*[iter(pathList)]*2))
			self.pathSegs = np.diff(self.path, axis=0)
			self.pathSegLenSq = np.sum(self.pathSegs*self.pathSegs, axis=1)
			self.pathSet = True

		def updateControl(self, message):
			if self.pathSet:
				self.position = np.array([message.pose.position.x, message.pose.position.y])

				# Find point along path which is nearest to robot
				t = np.clip( np.sum( (self.pose-self.path[:-1])*self.pathSegs, axis=1 ), 0, 1)
				projection = self.pathSegs[:-1] + (t*self.pathSegs)
				dist = np.linalg.norm(self.pose - projection, axis=1)
				closestPointIndex = np.argmin(dist)
				closestPoint = self.path[closestPointIndex]

				# Find intersection between circle around car and forward path from closestPoint
				pointTracker = closestPointIndex
				goal = None
				while goal == None:
					p1 = self.path[pointTracker]
					v = self.path[pointTracker+1] - self.path[pointTracker]
					a = np.dot(v, v)
					b = 2 * np.dot(v, (p1-self.position))
					c = np.dot(p1, p1) + np.dot(self.position, self.position)\
						- 2 * np.dot(p1, self.position) - self.Lfw**2

					disc = b**2 - 4*a*c
					if disc < 0:
						goal = None

	def setPath(self, message):
		self.path = np.array(zip(*[iter(message.data)]*2))
		self.pathSegs = np.diff(self.path, axis=0)
		self.pathSegLenSq = np.sum(self.pathSegs*self.pathSegs, axis=1)
		self.pathSet = True

	def updateControl(self, message):
		if self.pathSet:
			self.position = np.array([message.pose.position.x, message.pose.position.y])

			# Find point along path which is nearest to robot
			t = np.clip( np.sum( (self.position-self.path[:-1])*self.pathSegs, axis=1 ), 0, 1)
			# projection = self.pathSegs[:-1] + (t[:, None] * self.pathSegs)
			projection = self.pathSegs + (t[:, None] * self.pathSegs)
			dist = np.linalg.norm(self.position - projection, axis=1)
			closestPointIndex = np.argmin(dist)
			closestPoint = self.path[closestPointIndex]

			# Find intersection between circle around car and forward path from closestPoint
			pointTracker = closestPointIndex
			goal = None
			while goal == None:
				p1 = self.path[pointTracker]
				v = self.path[pointTracker+1] - self.path[pointTracker]
				a = np.dot(v, v)
				b = 2 * np.dot(v, (p1-self.position))
				c = np.dot(p1, p1) + np.dot(self.position, self.position)\
					- 2 * np.dot(p1, self.position) - self.Lfw**2

				disc = b**2 - 4*a*c
				if disc < 0:
					goal = None
				else:
					sqrt_disc = math.sqrt(disc)
					t1 = (-b + sqrt_disc) / (2*a)
					t2 = (-b - sqrt_disc) / (2*a)

					if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
						goal=  None
					else:
						t = max(0, min(1, -b/(2*a)))
						goal = p1+(t*v)

				pointTracker = pointTracker + 1

			# Compute driving command and publish
			point1 = self.position
			point2 = self.path[pointTracker-1]
			point3 = self.path[pointTracker]
			dist1 = np.linalg.norm(point2 - point1)
			dist2 = np.linalg.norm(point3 - point2)
			dist3 = np.linalg.norm(point1 - point3)
			s = (dist1 + dist2 + dist3) / 2
			k = math.sqrt(s*(s-dist1)*(s-dist2)*(s-dist3))
			curvature = (dist1*dist2*dist3)/(4*k)

			steering_angle = np.arctan(curvature*self.distWheels)

			msg = AckermannDriveStamped()
			msg.drive.steering_angle = -1.0*steering_angle
			msg.drive.speed = self.speed

			self.pub.publish(msg)

if __name__=="__main__":
	# Tell ROS that we're making a new node.
	rospy.init_node("Pure_Pursuit_Node")

	# Init the node
	PurePursuit()

	# Don't let this script exit while ROS is still running
	rospy.spin()