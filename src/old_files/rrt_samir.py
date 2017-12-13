#!/usr/bin/python
"""
Created on Sun Apr 11 

@author: samir and martian
"""

import random
import numpy as np
import math
import time
import utils as Utils
from matplotlib import pyplot as plt
from matplotlib import cm
from scipy.misc import imread

#Messages
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap

class Node(object):
    def __init__(self, point, parent=None, cost=0):
        self.point = point
        self.parent = parent
        self.cost = cost

class RRT(object):
    def __init__(self, environment):
        self.start_flag = False
        self.goal_flag = False
        self.start = None
        self.goal = None
        self.re_init = True

        #Start and Goal Node Subscriber
        self.inferred_pose_sub = rospy.Subscriber("/infered_pose_map", PoseStamped, self.set_inferred_pose, queue_size=1)
        self.goal_sub = rospy.Subscriber("/goal_point", PoseStamped, self.set_goal, queue_size=1)
        #Pure Pursuit Publisher
        self.pub_control = rospy.Publisher("pursuit_control_input",Float32MultiArray, queue_size=1)

        self.map_loc = environment
        self.start_loc = start
        self.goal_loc = goal
        self.goal_loc_np = np.array(goal)

        # Tune for RRT #
        self.max_dist = 20 
        self.min_dist = 1
        self.goal_radius = 20
        self.NUMNODES = 100000
        self.goal_check_period = 50
        self.goal_check_radius = 10

        # Load and store map #
        self.map_img = imread(self.map_loc)
        self.mapWidth = len(self.map_img[0])
        self.mapHeight = len(self.map_img)
        self.open_cells = self.load_map(self.map_img)
        self.open_cells = 
        self.open_cells_tuple = tuple(self.open_cells)

        # Tracker variables #
        self.max_dist_sq = self.max_dist**2
        self.min_dist_sq = self.min_dist**2
        self.goal_check_radius_sq = self.goal_check_radius**2
        self.nodes = list()
        self.nodeLocs = np.array(self.start_loc, ndmin=2)
        self.nodeDict = dict()
        self.solutionPath = None
        self.count = 0
        self.pathFound = False

    def wall(self):
        vector = (self.start[0] - self.goal[0], self.start[1] - self.goal[1])
        slope = vector[1]/vector[2]

        angle = math.atan2(vector[1],vector[2])
        perpendic = self.start_orientation_angle - np.pi/2.0




    def set_inferred_pose(self, msg):
        if not self.start_flag:
            self.start = (msg.pose.position.x, msg.pose.position.y)
            self.start_orientation = Utils.quaternion_to_angle(msg.pose.orientation)
            self.start_flag = True
    
    def set_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.start_flag = False

    def load_map(self, map_img):
        cells = set()

        for i in range(len(map_img[0])):
            for j in range(len(map_img)):
                if map_img[i][j] == 255:
                    cells.add((j,i))

        return cells

    def checkCollision(self, pA, pB):
        '''
        Takes two numpy arrays
        '''
        if pB[0] < pA[0]:
            pointA = pB
            pointB = pA
        else:
            pointA = pA
            pointB = pB
        diff = pointB - pointA

        if diff[0] < 1:

            #slope = diff[0]/diff[1]
            y_range = np.arange(1, abs(diff[1]))
            x_range = np.tile(pointA[0], (len(y_range), 1))
            vals = np.array(zip(x_range, y_range))
            try:
                cells = np.concatenate(vals, np.array([pointA, pointB]))
            except:
                return True
            #return ((tuple(pointA) in self.open_cells) and (tuple(pointB) in self.open_cells))

        else:
            slope = diff[1]/float(diff[0])
            x_range = np.arange(1, diff[0])
            y_range = slope * x_range + pointA[1]
            x_vals = x_range + pointA[0]
            y_low = np.floor(y_range)
            y_low = np.array(zip(x_vals, y_low))
            y_high = np.ceil(y_range)
            y_high = np.array(zip(x_vals, y_high))
            try:
                cells = np.concatenate((y_low, y_high, np.array([pointA, pointB])))
            except:
                return True

        # collision flag
        collision = False

        for cell in cells:
            test_cell = (int(cell[0]), int(cell[1]))
            if test_cell not in self.open_cells:
                collision = True
                break

        return collision

    def steerPoint(self, pointA, pointB):
        '''
        pointA is nearest node, pointB is random node
        returns new node loc as numpy array
        '''
        # Find length to extend by
        magSq = self.distSquared(pointA, pointB)
        mag = np.sqrt(magSq)
        if magSq > self.max_dist_sq:
            length = self.max_dist
        elif magSq  < self.min_dist_sq:
            return None, 0
        else:
            length = mag
        
        # Compute unit vector from A -> B
        diff = pointB - pointA
        diffNorm = diff/mag

        # Scale unit vector and add to A
        return pointA + length*diffNorm, length

    def distSquared(self, pointA, pointB):
        '''
        pointA is either vector (point) or matrix (list of points) // pointB is single vector (point)
        '''
        dim = len(pointA.shape)-1
        seg = pointA - pointB
        return np.sum(seg * seg, axis=dim)

    def randomPoint(self):
        if self.count % self.goal_check_period != 0:
            return random.choice(self.open_cells_tuple)
        else:
            return self.goal_loc

    def nearest(self, pointA):
        '''
        Returns a numpy array
        '''
        dists = self.distSquared(self.nodeLocs, pointA)
        idx = np.argmin(dists)
        return self.nodeLocs[idx]

    def createSolution(self, goalNode):
        self.solutionPath = [goalNode]
        node = goalNode
        while node is not None:
            parentNode = node.parent
            if parentNode is not None:
                self.solutionPath.append(parentNode)
            node = parentNode

        self.solutionPath.reverse()
        self.plotSolution()

    def goalCheck(self, pointLoc):
        distSq = self.distSquared(self.goal_loc_np, pointLoc)
        if distSq < self.goal_check_radius_sq:
            return True
        return False

    def plotSolution(self):
        # Plot map image onto figure
        fig = plt.gcf()
        fig.clf()
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(self.map_img, cmap=cm.Greys_r)
        ax.axis('image')
        plt.draw()
        
        # Plot initial point and goal point
        initial_circle = plt.Circle(self.start_loc, radius = 10, color = 'g')
        goal_circle = plt.Circle(self.goal_loc, radius = 10, color = 'r')
        ax.add_patch(initial_circle)
        ax.add_patch(goal_circle)
        plt.draw()

        # Plot solution
        for node in self.solutionPath:
            if node.parent is not None:
                ax.plot([node.point[0], node.parent.point[0]],[node.point[1], node.parent.point[1]], color = 'r', linestyle = '-', linewidth = 2)
                plt.draw()
        plt.show()
        self.pathFound = True

    def plotAll(self):
        # Plot map image onto figure
        fig = plt.gcf()
        fig.clf()
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(self.map_img, cmap=cm.Greys_r)
        ax.axis('image')
        plt.draw()
        
        # Plot initial point and goal point
        initial_circle = plt.Circle(self.start_loc, radius = 10, color = 'g')
        goal_circle = plt.Circle(self.goal_loc, radius = 10, color = 'r')
        ax.add_patch(initial_circle)
        ax.add_patch(goal_circle)
        plt.draw()

        # Plot all
        for node in self.nodes:
            if node.parent is not None:
                ax.plot([node.point[0], node.parent.point[0]],[node.point[1], node.parent.point[1]], color = 'b', linestyle = '-', linewidth = 2)
                plt.draw()
        plt.show()

    def findPath(self):
        # Initialize start node
        startNode = Node(np.array(self.start_loc))
        self.nodes.append(startNode)
        self.nodeDict[self.start_loc] = startNode

        # Goal flag
        searching = True

        while searching:
            if self.count >= self.NUMNODES:
                searching = False
            
            x_rand = self.randomPoint() # Numpy array
            x_nearest = self.nearest(x_rand) # Numpy array
            x_new, cost = self.steerPoint(x_nearest, x_rand) # Numpy array, and scalar
            if x_new is not None:
                if not self.checkCollision(x_nearest, x_new):
                    parentNode = self.nodeDict[tuple(x_nearest)]
                    newNode = Node(x_new, parent=parentNode, cost=parentNode.cost + cost)
                    # Add to trackers
                    self.nodes.append(newNode)
                    self.nodeLocs = np.vstack([self.nodeLocs, x_new])
                    self.nodeDict[tuple(x_new)] = newNode

                    # Goal check
                    if self.goalCheck(x_new):
                        searching = False
                        self.solutionPath = self.createSolution(newNode)

            self.count += 1
        if not self.pathFound:
            self.plotAll()
        return self.count


if __name__ == '__main__':
    rospy.init_node("path_planner")

    test = RRT('../maps/basement_fixed_more_dilated.png')
    # if test.re_init:
    test.findPath()
    rospy.spin()