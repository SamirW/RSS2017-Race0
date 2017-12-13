# -*- coding: utf-8 -*-
"""
Created on Sun Apr  9 08:03:37 2017

@author: clemmie
"""
from math import *
# import rospy
# import std_msgs.msg
import random
import numpy as np
import sys
from matplotlib import pyplot as plt
from matplotlib import cm
from scipy.misc import imread
# from std_msgs.msg import Float32, Float32MultiArray  # the rostopic message we subscribe
# from ackermann_msgs.msg import AckermannDriveStamped
import time

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent
    def getPoint(self):
        return self.point

class rrtstar(object):
    def __init__(self):
#        # Init subscribers and publishers
#        self.sub = rospy.Subscriber("/scan", LaserScan,\
#                self.safetyCB, queue_size=1)
#                
#        self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety",\
#                AckermannDriveStamped, queue_size =1 )
        # subscribe to the published goal point
        #self.goal_sub = rospy.Subscriber("/goal_point",PoseStamped, self.set_goal, queue_size=1)
        # subscribe to initial car position
        #self.map_pose_sub = rospy.Subscriber("/infered_pose_map", PoseStamped, self.set_initial_pose, queue_size = 1)

        self.goal_point = None
        self.initial_pose = None
        
        # Map image dimensions
        self.XDIM = 1300
        self.YDIM = 1300
        # Settings which can be tuned
        self.delta = 10.0
        self.GOAL_RADIUS = 10
        self.NUMNODES = 5000
        # Map details
        self.MAP_NAME = '../maps/basement_fixed_more_dilated.png'
        self.map_img = imread(self.MAP_NAME)
        self.available_cells = set()
        self.available_cells_tuple = None
        self.count = 0
        self.i = 0

    def read_map(self):
        print "entered read map"
        begin = time.time()
        map_img = imread('../maps/basement_fixed_more_dilated.png')
        for i in range(len(map_img)):
            for j in range(len(map_img[0])):
                if map_img[i][j] == 255:
                    self.available_cells.add((i,j))
        print time.time() - begin
        self.available_cells_tuple = tuple(self.available_cells)
        print "max size is "
        print sys.maxsize
        print "tuple size is "
        print len(self.available_cells_tuple)
        print "set size is "
        print len(self.available_cells)

    def set_goal(self,msg):
        pose = msg.pose
        x = pose.position.x
        y = pose.position.y 
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        #self.goal_point = (x,y)

    def set_initial_pose(self,msg):
        pose = msg.pose
        x = pose.position.x
        y = pose.position.y
        orientation = pose.orientation
        self.initial_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.orientation)
        #self.initial_pose = (x,y,orientation)


    # Calculate the distance between two points, a and b
    def dist(self, a,b):
        return sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

    def goal_collision(self, a, b, radius):
        distance = self.dist(a,b)
        # If the tree has reached within the allowable "goal area" we have collided with the goal point
        if (distance <= radius):
            return True
        return False

    def step_from_to(self, a,b):
        if self.dist(a,b) < self.delta:
            return b
        else:
            theta = atan2(b[1]-a[1],b[0]-a[0])
            return a[0] + self.delta*cos(theta), a[1] + self.delta*sin(theta)

    def random_point(self):
        if self.i % 100 == 0:

            while True:
                occupied = True
                p = int(random.random()*self.XDIM), int(random.random()*self.YDIM)
                if (p[1], p[0]) in self.available_cells:
                    print p
                    return p

            # q = random.choice(self.available_cells_tuple)
            # p = float(q[0]), float(q[1])

            # print p
            # return p
        else:
            return goalPoint.getPoint()
        
    def find_obstacle():
        pass

    def main(self):
        print "in main"
        global map_img
        
        # Get these from Localization Node.
        # initial_coords = [66,265]
        initial_coords = [476, 327]
        #goal_coords = [300, 300]
        # goal_coords = [347, 847]
        goal_coords = [1170, 319]
        # Set initial point and goal nodes
        self.initialPoint = Node(initial_coords,None)
        self.goalPoint = Node(goal_coords, None)
        
        # Set current state to be create the random tree (root at the start point)
        currentState = 'buildTree'
        
        # Plot map image onto figure
        fig = plt.gcf()
        fig.clf()
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(self.map_img, cmap=cm.Greys_r)
        ax.axis('image')
        plt.draw()
        
        # Plot initial point and goal point
        initial_circle = plt.Circle(initial_coords, radius = 10, color = 'g')
        goal_circle = plt.Circle(goal_coords, radius = 10, color = 'r')
        ax.add_patch(initial_circle)
        ax.add_patch(goal_circle)
        plt.draw()

        # Initialoze variables
        nodes = []
        solution_path = []

        nodes.append(self.initialPoint)
        start = time.time()
        for i in range(self.NUMNODES):
            if currentState == 'goalFound':
                currNode = goalNode.parent
                print "Goal Reached"
                for node in nodes:
                    if node.parent is not None:
                        ax.plot([node.point[0], node.parent.point[0]],[node.point[1], node.parent.point[1]], color = 'b', linestyle = '-', linewidth = 2)
                        plt.draw()
                # Plot solution path in red and pass this on to the controller.
                while currNode.parent != None:
                    solution_path.append(currNode.point[0])
                    solution_path.append(currNode.point[1])
                    ax.plot([currNode.point[0], currNode.parent.point[0]],[currNode.point[1], currNode.parent.point[1]], color = 'r', linestyle = '-', linewidth = 2)
                    plt.draw()
                    currNode = currNode.parent
                print "complete"
                print time.time()-start
                plt.show()
                solution_path.reverse()
                print solution_path
                return solution_path

                
            elif currentState == 'buildTree':
                # Check that we haven't created too many nodes
                foundNext = False
                # If we haven't found our next point, then generate more random points until it works out.
                while foundNext == False:
                    rand = self.random_point()
                    #rand2 = self.random_point()
                    #rand3 = self.random_point()
                    parentNode = nodes[0]
                    # If the distance from the node to the random point is less than that from the parent node
                    # to the random point, then create a new point. 
                    for p in nodes:
                        if self.dist(p.point,rand) <= self.dist(parentNode.point,rand): # and self.dist(p.point,rand2) <= self.dist(parentNode.point,rand2) and self.dist(p.point,rand3) <= self.dist(parentNode.point,rand3):
                            newPoint = self.step_from_to(p.point,rand)
                            #newPoint2 = self.step_from_to(p.point,rand2)
                            #newPoint3 = self.step_from_to(p.point,rand3)
                            # If the new points are unoccupied (ie white), let point p be the new parent.
                            if (int(newPoint[1]), int(newPoint[0])) in self.available_cells: # and self.map_img[int(newPoint2[1])][int(newPoint[0])] == 255 and self.map_img[int(newPoint3[1])][int(newPoint[0])] == 255:
                                parentNode = p
                                foundNext = True

                # Plot all the nodes and lines connecting them for each different tree in different colors.
                newnode = self.step_from_to(parentNode.point,rand)
                #newnode2 = self.step_from_to(parentNode.point,rand2)
                #newnode3 = self.step_from_to(parentNode.point,rand3)
                nodes.append(Node(newnode, parentNode))
                #nodes.append(Node(newnode2, parentNode))
                #nodes.append(Node(newnode3, parentNode))
                #ax.plot([parentNode.point[0], newnode[0]], [parentNode.point[1], newnode[1]], color = 'blue', linestyle = '-', linewidth = 2)
                #plt.draw()
                #ax.plot([parentNode.point[0], newnode[0]], [parentNode.point[1], newnode2[1]], color = 'g', linestyle = '-', linewidth = 2)
                #plt.draw()
                #ax.plot([parentNode.point[0], newnode[0]], [parentNode.point[1], newnode3[1]], color = 'pink', linestyle = '-', linewidth = 2)
                #plt.draw()


                # If tree collides with goal point, goal has been reached
                if self.goal_collision(newnode, self.goalPoint.point, self.GOAL_RADIUS): # or self.goal_collision(newnode2, self.goalPoint.point, self.GOAL_RADIUS) or self.goal_collision(newnode3, self.goalPoint.point, self.GOAL_RADIUS):
                    currentState = 'goalFound'
                    goalNode = nodes[len(nodes)-1]

        for node in nodes:
            if node.parent is not None:
                ax.plot([node.point[0], node.parent.point[0]],[node.point[1], node.parent.point[1]], color = 'r', linestyle = '-', linewidth = 2)
                plt.draw()
        plt.show()
        return "timed out: tried to create too many nodes"

if __name__ == '__main__':
    rrtstariter = rrtstar()
    rrtstariter.read_map()
    rrtstariter.main()

        
    #    # Tell ROS that we're making a new node.
    #    rospy.init_node("Safety_Node")
    #
    #    # Init the node
    #    Safety()
    #
    #    # Don't let this script exit while ROS is still running
    #    rospy.spin()
