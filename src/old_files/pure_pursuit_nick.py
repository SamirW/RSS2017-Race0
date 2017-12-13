#!/usr/bin/env python
import rospy
import numpy as np
import math
import utils as Utils

import tf.transformations
import tf

from matplotlib import pyplot as plt
from std_msgs.msg import String, Header, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
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

        # Output path for RViz
        self.pathPub = rospy.Publisher("path_pub", Path, queue_size=1)

        # transform
        self.pub_tf = tf.TransformBroadcaster()

        rospy.loginfo("Pure pursuit node initialized")

        # Set parameters
        self.steering_gain = 5
        self.speed = 1
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
        self.goalReachedFlag = False
        self.prevPose = None
        self.eta = None

        self.numLocalizationCalls = 0

    def setPath(self, message):
        pathList = message.data
        self.path = np.array(zip(*[iter(pathList)]*2))
        self.goalLoc = self.path[-1]
        self.pathSegs = np.diff(self.path, axis=0)
        self.pathSegLenSq = np.sum(self.pathSegs*self.pathSegs, axis=1)
        print self.path
        self.pathSet = True

        # Create path to publish to rviz
        rvizPath = Path()
        poseList = []
        for loc in self.path:
            tempPose = PoseStamped()
            tempPose.pose.position.x = loc[0]
            tempPose.pose.position.y = loc[1]
            poseList.append(tempPose)
        rvizPath.poses = poseList

        self.pathPub.publish(rvizPath)
        self.pub_tf.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now() , "/base_link", "/map")
        print "path set", len(self.path)

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

        while theta_local < -np.pi:
            theta_local += 2*np.pi
        while theta_local > np.pi:
            theta_local -= 2*np.pi

        self.steering_angle = (0.35/(2*np.pi))*theta_local
        print self.steering_angle/0.35

        # print "current_loc is ", self.pose 
        # print "goal_loc is ", loc
        # print "self.orientation is ", self.orientation
        # print "theta is " + str(theta)
        # print "steering angle is " + str(self.steering_angle)

    def purePursuit(self, goalLoc):
        goalSeg = goalLoc - self.pose
        L1 = np.linalg.norm(goalSeg)
        vertSeg = np.array([0, 1])

        inside = np.dot(goalSeg, vertSeg)/(np.linalg.norm(goalSeg)*np.linalg.norm(vertSeg))
        self.eta = np.arccos(inside) - self.orientation
        self.steering_angle = (math.atan2(2*self.L*math.sin(self.eta), L1)/np.pi*0.35)
        print "current_loc is ", self.pose
        print "goal_loc is ", goalLoc
        print "self.orientation is ", self.orientation
        print "eta is " + str(self.eta)
        print "steering angle is " + str(self.steering_angle)

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
        # self.numLocalizationCalls += 1
        # if not self.numLocalizationCalls % 5 == 0:
        #     pass

        # else:
        if self.pathSet:
            # Part 0 - Init position and orientation
            self.pose = np.array([message.pose.position.x, message.pose.position.y])
            self.orientation = Utils.quaternion_to_angle(message.pose.orientation)

            currPointLoc = self.path[self.pointCounter]
            currDist = self.dist(currPointLoc, self.pose)
            print "Distance: ", currDist

            self.purePursuit(currPointLoc)
            if currDist < 10 or self.eta > np.pi/2 or self.eta < -np.pi/2:
            	if self.pointCounter != (len(self.path)-1):
            		self.pointCounter = self.pointCounter + 1
            		print "new node acquired"
            	else:
                    self.speed = 0
                    print "goal reached"
				# self.pathSet = False

  #           # Publish message
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = -self.steering_angle
            msg.drive.speed = self.speed
            print self.speed, "speed"

            self.pub.publish(msg)



if __name__ == '__main__':
    # Tell ROS that we're making a new node.
    rospy.init_node("Pure_Pursuit_Node")

    # Init the node
    PurePursuit()

    # Don't let this script exit while ROS is still running
    rospy.spin()
