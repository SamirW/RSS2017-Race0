#!/usr/bin/env python
import rospy
import numpy as np
import math
import utils_TA as Utils

import tf.transformations
import tf

from matplotlib import pyplot as plt
from std_msgs.msg import String, Header, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped

from visualization_msgs.msg import Marker

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

        # Output path for RViz
        self.pathPub = rospy.Publisher("path_pub", Path, queue_size=1)
        
        # Output pure pursuit points for RViz
        self.viz_namespace = "/pure_pursuit"
	self.nearest_point_pub = rospy.Publisher(self.viz_namespace + "/nearest_point", Marker, queue_size = 1)
	self.lookahead_point_pub = rospy.Publisher(self.viz_namespace + "/lookahead_point", Marker, queue_size = 1)

        # transform
        self.pub_tf = tf.TransformBroadcaster()

        rospy.loginfo("Pure pursuit node initialized")

        # Set parameters
        self.steering_gain = 5
        self.speed = 4
        self.steering_angle = 0
        self.Lfw = self.steering_gain * self.speed  # Look ahead distance
        self.Lfw2 = self.Lfw**2  # Look ahead distance ^ 2
        self.path = None
        self.pathSegs = None
        self.goalLoc = None
        self.pathSet = False
        self.pathSegLenSq = None
        self.pose = None
        self.orientation = None
        self.distWheels = 1.05 # feet
        self.L = 65.09602966* self.distWheels # pixels
        self.pointCounter = 1
        

        self.do_viz = True
	self.trajectory = None
	self.lookahead = 5	#Should be less than max reacquire distance
	self.max_reacquire = 8
        
    def visualize(self):
	''' Publishes visualization topics:
		- Circle to indicate the nearest point along the trajectory
		- Circle to indicate the chosen lookahead point
	'''
	if not self.do_viz:
		return
	# visualize: pure pursuit circle, lookahead intersection, lookahead radius line, nearest point
	if self.nearest_point_pub.get_num_connections() > 0 and isinstance(self.nearest_point, np.ndarray):
		self.nearest_point_pub.publish(Utils.make_circle_marker(
			self.nearest_point, 0.5, [0.0,0.0,1.0], "/map", self.viz_namespace, 0, 3))

	if self.lookahead_point_pub.get_num_connections() > 0 and isinstance(self.lookahead_point, np.ndarray):
		self.lookahead_point_pub.publish(Utils.make_circle_marker(
			self.lookahead_point, 0.5, [1.0,1.0,1.0], "/map", self.viz_namespace, 1, 3))

    def setPath(self, message):
        pathList = message.data
        self.path = np.array(zip(*[iter(pathList)]*2))
        self.goalLoc = self.path[-1]
        self.pathSegs = np.diff(self.path, axis=0)
        self.pathSegLenSq = np.sum(self.pathSegs*self.pathSegs, axis=1)
        self.pathSet = True

        # Create path to publish to rviz
        rvizPath = Path()
        poseList = []
	#Create trajectory list of (x,y) for use in pure pursuit
	self.trajectory = []
        for loc in self.path:
            tempPose = PoseStamped()
            tempPose.pose.position.x = loc[0]
            tempPose.pose.position.y = loc[1]
            poseList.append(tempPose)
	    self.trajectory.append((loc[0],loc[1]))
        rvizPath.poses = poseList

        self.pathPub.publish(rvizPath)
        self.pub_tf.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now() , "/base_link", "/map")


    def distSq(self, pointA, pointB):
        '''
        pointA is either vector (point) or matrix (list of points) // pointB is single vector (point)
        '''
        dim = len(pointA.shape)-1
        seg = pointA - pointB
        return np.sum(seg * seg, axis=dim)

    def dist(self, pointA, pointB):
        return np.sqrt(self.distSq(pointA, pointB))

    def driveTowardNode(self, loc):
        # goalSeg = loc - self.pose
        # vertSeg = np.array([0, 1])

        # inside = np.dot(goalSeg, vertSeg)/(np.linalg.norm(goalSeg)*np.linalg.norm(vertSeg))
        # theta = np.arccos(inside) - self.orientation
        # self.steering_angle = (-theta)*0.7/(np.pi/2) - (20*np.pi/180.0)

        goalSeg = loc-self.pose
        theta_global = np.arctan2(goalSeg[1], goalSeg[0])
        theta_local = theta_global - self.orientation

        if theta_local < -2*np.pi:
        	theta_local += 2*np.pi
        elif theta_local > 2*np.pi:
        	theta_local -= 2*np.pi

        self.steering_angle = (0.35/(2*np.pi))*theta_local

        # print "current_loc is ", self.pose 
        # print "goal_loc is ", loc
        # print "self.orientation is ", self.orientation
        # print "theta is " + str(theta)
        # print "steering angle is " + str(self.steering_angle)

    def purePursuit(self, pose):

	# step 1
	nearest_point, nearest_dist, t, i = Utils.nearest_point_on_trajectory(pose[:2], self.trajectory.np_points)
	self.nearest_point = nearest_point

	if nearest_dist < self.lookahead:
		# step 2
		lookahead_point, i2, t2 = \
			Utils.first_point_on_trajectory_intersecting_circle(pose[:2], self.lookahead, self.trajectory.np_points, i+t, wrap=self.wrap)
		if i2 == None:
			self.lookahead_point = None
		else:
			self.lookahead_point = lookahead_point

	elif nearest_dist < self.max_reacquire:
		self.lookahead_point = self.nearest_point

	else:
		self.lookahead_point = None

	# stop of there is no navigation target, otherwise use ackermann geometry to navigate there
	if not isinstance(self.lookahead_point, np.ndarray):
		self.stop()
	else:
		steering_angle = self.determine_steering_angle(pose, self.lookahead_point)
		# send the control commands
		self.apply_control(self.speed, steering_angle)

	self.visualize()

    def plot(self):
        fig = plt.gcf()
        fig.clf()
        ax = fig.add_subplot(1, 1, 1)

        # Plot initial point and goal point
        initial_circle = plt.Circle(self.pose, radius = 1, color = 'g')
        ax.add_patch(initial_circle)
        plt.draw()

        # plot look ahead distance
        new_circle = plt.Circle(self.pose, radius = 1+self.Lfw, color = 'b', Fill=False)
        ax.add_patch(new_circle)
        plt.draw()

        for i in range(len(self.path)-1):
            ax.plot([self.path[i+1][0], self.path[i][0]],[self.path[i+1][1], self.path[i][1]], color = 'r', linestyle = '-', linewidth = 2)
            plt.draw()

        plt.show()

    def updateControl(self, message):
        if self.pathSet:
            # Part 0 - Init position and orientation
            self.pose = np.array([message.pose.position.x, message.pose.position.y])
            self.orientation = Utils.quaternion_to_angle(message.pose.orientation)

            self.currPointLoc = self.path[self.pointCounter]
	    self.lastPointLoc = self.path[self.pointCounter-1]
            currDist = self.dist(self.currPointLoc, self.pose)
            print "Distance: ", currDist
            if currDist > 10:
                self.driveTowardNode(self.currPointLoc)
            else:
		self.purePursuit(self.lastPointLoc,self.currPointLoc)
		if currDist < 0.2:
                	if self.pointCounter != (len(self.path)-1):
                    		self.pointCounter = self.pointCounter + 1
                    		print "new node acquired"
                	else:
                    		print "goal reached"
				self.pathSet = False

            # Publish message
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = self.steering_angle
            msg.drive.speed = self.speed

            self.pub.publish(msg)

    def determine_steering_angle(self, pose, lookahead_point):
	'''
	Given a robot pose, and a lookahead point, determine the open loop control 
	necessary to navigate to that lookahead point. Uses Ackermann steering geometry.
	'''
	# get the lookahead point in the coordinate frame of the car
	rot = utils.rotation_matrix(-pose[2])
	delta = np.array([lookahead_point - pose[0:2]]).transpose()
	local_delta = (rot*delta).transpose()
	local_delta = np.array([local_delta[0,0], local_delta[0,1]])
	# use the ackermann model
	steering_angle = self.model.steering_angle(local_delta)
	return steering_angle

    def apply_control(self, speed, steering_angle):
	self.actual_speed = speed
	drive_msg_stamped = AckermannDriveStamped()
	drive_msg = AckermannDrive()
	drive_msg.speed = speed
	drive_msg.steering_angle = steering_angle
	drive_msg.acceleration = 0
	drive_msg.jerk = 0
	drive_msg.steering_angle_velocity = 0
	drive_msg_stamped.drive = drive_msg
	self.pub.publish(drive_msg_stamped)

    def stop(self):
	print "Stopping"
	drive_msg_stamped = AckermannDriveStamped()
	drive_msg = AckermannDrive()
	drive_msg.speed = 0
	drive_msg.steering_angle = 0
	drive_msg.acceleration = 0
	drive_msg.jerk = 0
	drive_msg.steering_angle_velocity = 0
	drive_msg_stamped.drive = drive_msg
	self.control_pub.publish(drive_msg_stamped)

if __name__ == '__main__':
    # Tell ROS that we're making a new node.
    rospy.init_node("Pure_Pursuit_Node")

    # Init the node
    PurePursuit()

    # Don't let this script exit while ROS is still running
    rospy.spin()
