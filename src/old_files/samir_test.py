#!/usr/bin/env python
import rospy
import numpy as np
import math
import Utils

from matplotlib import pyplot as plt
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
        self.steering_gain = 5.0
        self.speed = 2.5
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

    def setPath(self, message):
            pathList = message.data
            self.path = np.array(zip(*[iter(pathList)]*2))
            self.goalLoc = self.path[-1]
            self.pathSegs = np.diff(self.path, axis=0)
            self.pathSegLenSq = np.sum(self.pathSegs*self.pathSegs, axis=1)
            self.pathSet = True

    def distSq(self, pointA, pointB):
        '''
        pointA is either vector (point) or matrix (list of points) // pointB is single vector (point)
        '''
        dim = len(pointA.shape)-1
        seg = pointA - pointB
        return np.sum(seg * seg, axis=dim)

    def dist(self, pointA, pointB):
        return np.sqrt(self.distSq(pointA, pointB))

    def driveTowardsNode(self, loc):
        goalSeg = self.pose - loc
        vertSeg = np.array([0, 1])

        inside = np.dot(goalSeg, vertSeg)/(np.linalg.norm(goalSeg)*np.linalg.norm(vertSeg))
        theta = np.arccos(inside) - self.orientation
        self.driving_angle = -theta

    def purePursuit(self, goalLoc):
        goalSeg = self.pose - goalLoc
        L1 = np.linalg.norm(goalSeg)
        vertSeg = np.array([0, 1])

        inside = np.dot(goalSeg, vertSeg)/(np.linalg.norm(goalSeg)*np.linalg.norm(vertSeg))
        eta = np.arccos(inside) - self.orientation
        self.driving_angle = math.atan2(2*self.L*math.sin(eta), L1)

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

    def updateControl(self):
        if self.pathSet:
            # Part 1 - Find closest node
            distToNodesSq = self.distSq(self.path, self.pose)

            # Check if at node
            if len(np.argwhere(distToNodesSq == 0)) > 0:
                closestNode = self.pose
                closestNodeIndex = np.argwhere(distToNodesSq==0).flatten()[0]
            # else, find closest node
            else:
                t = np.clip( np.sum( (self.pose-self.path[:-1])*self.pathSegs, axis=1 ), 0, 1)
                projection = self.path[:-1] + (t[:, None]*self.pathSegs)
                distToSegments = self.dist(projection, self.pose)
                closestNodeIndex = np.argmin(distToSegments)
                closestNode = self.path[closestNodeIndex]

            # Too close to goal, go straight to it
            distToGoalSq = self.distSq(self.pose, self.goalLoc)
            if distToGoalSq < self.Lfw2:
                # If within radius, stop
                if distToGoalSq < 2000:
                    self.speed = 0
                # Go straight to goal
                self.driveTowardsNode(self.goalLoc)

            # Check line segments starting from closestNode
            # Find intersection between circle around car and forward path from closestPoint
            else:
                pointTracker = closestNodeIndex
                goal = None
                while goal is None:
                    if pointTracker > len(self.path)-2:
                        break
                    p1 = self.path[pointTracker]
                    v = self.path[pointTracker+1] - self.path[pointTracker]
                    a = np.dot(v, v)
                    b = 2 * np.dot(v, (p1-self.pose))
                    c = np.dot(p1, p1) + np.dot(self.pose, self.pose)\
                        - 2 * np.dot(p1, self.pose) - self.Lfw2

                    disc = b**2 - 4*a*c
                    if disc < 0:
                        goal = None
                    else:
                        sqrt_disc = math.sqrt(disc)
                        t1 = (-b + sqrt_disc) / float((2*a))
                        t2 = (-b - sqrt_disc) / float((2*a))

                        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
                            goal =  None
                        else:
                            if not (0 <= t1 <= 1) and (0 <= t2 <= 1):
                                t = t2
                            elif (0 <= t1 <= 1) and not (0 <= t2 <= 1):
                                t = t1
                            else:
                                t = min(t1, t2)
                            goal = p1+(t*v)

                    pointTracker = pointTracker + 1

                # If no goal found
                if goal is None:
                    self.driveTowardsNode(closestNode)
                else: # Pure pursuit
                    self.purePursuit(goal)

            # Publish message
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = self.steering_angle
            msg.drive.speed = self.speed

            self.pub.publish(msg)

if __name__ == '__main__':
    # Tell ROS that we're making a new node.
    rospy.init_node("Pure_Pursuit_Node")

    # Init the node
    PurePursuit()

    # Don't let this script exit while ROS is still running
    rospy.spin()
