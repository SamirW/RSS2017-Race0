# -*- coding: utf-8 -*-
"""
Created on Sun Apr  9 08:03:37 2017

@author: clemmie
"""

import random
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm
from scipy.misc import imread

##### General thought: I don't actually see a collision checker in here other than for the goal? #####

class rrtstar(object):
    def __init__(self, point, parent):
        ##### Let's talk about how to use the idea of nodes and parents without #####
        ##### creating a class for them - maybe careful handling of np arrays?  #####
        super(Node, self).__init__()
        self.point = point
        self.parent = parent
        self.goal_point = None
        self.initial_pose = None
        
#        # Init subscribers and publishers
#        self.sub = rospy.Subscriber("/scan", LaserScan,\
#                self.safetyCB, queue_size=1)
#                
#        self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety",\
#                AckermannDriveStamped, queue_size =1 )
        # subscribe to the published goal point
        self.goal_sub = rospy.Subscriber("/goal_point",PoseStamped, self.set_goal, queue_size=1)
        # subscribe to initial car position
        self.map_pose_sub = rospy.Subscriber("/infered_pose_map", PoseStamped, self.set_initial_pose, queue_size = 1)


        # Map image dimensions
        self.XDIM = 1300
        self.YDIM = 1300
        # Settings which can be tuned
        self.delta = 30.0
        self.GOAL_RADIUS = 20
        self.NUMNODES = 5000
        # Map details
        MAP_NAME = 'basement_fixed.png'
        self.map_img = imread(MAP_NAME)

        count = 0

    def set_goal(self,msg):
        pose = msg.pose
        x = pose.position.x
        y = pose.position.y 
        self.goal_point = (x,y)

    def set_initial_pose(self,msg):
        pose = msg.pose
        x = pose.position.x
        y = pose.position.y
        orientation = pose.orientation
        self.initial_pose = (x,y,orientation)


    # Calculate the distance between two points, a and b
    ##### Use dist^2 for most calculations to avoid a square root #####
    ##### Also use np arrays #####
    def dist(a,b):
        return sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]))

    def goal_collision(a, b, radius):
        distance = dist(a,b)
        # If the tree has reached within the allowable "goal area" we have collided with the goal point
        if (distance <= radius): ##### distance^2 <= radius^2 ######
            return True
        return False

    def step_from_to(a,b):
        if dist(a,b) < self.delta: ##### distance^2 < self.delta^2 #####
            return b
        else:
            theta = atan2(b[1]-a[1],b[0]-a[0])  ##### Use vectors instead of trig #####
            return a[0] + self.delta*cos(theta), a[1] + self.delta*sin(theta) ##### Use vectors instead of trig ######

    def random_point():
        while True:
            occupied = True
            p = random.random()*self.XDIM, random.random()*self.YDIM
            ##### Unsure, but sounds like it will only return a random point when it's occupied? #####
            if(self.map_img[int(p[1])][int(p[0])][0] == 255): 
                occupied = False
            if occupied:
                return p
        
    def reset():
        global count
        count = 0

    def main():
        global count
        global map_img
        
        # Get these from Localization Node.
        initial_coords = [66,265]
        goal_coords = [300, 300]
        
        # Set initial point and goal nodes
        initialPoint = Node(initial_coords,None)
        goalPoint = Node(goal_coords, None)
        
        # Set current state to be create the random tree (root at the start point)
        currentState = 'buildTree'
        
        # Plot map image onto figure
        
        ##### You should create a plot only when you're done with the RRT* #####
        ### Useful to see what's going on with RRT*, but will be much faster ### 
        ##### if we wait to plot until the very end #####

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
        reset()
        nodes.append(initialPoint)

        while True:
            if currentState == 'goalFound':
                currNode = goalNode.parent
                print "Goal Reached"
                
                # Plot solution path in red and pass this on to the controller.
                while currNode.parent != None:
                    solution_path.append(currNode.point[0])
                    solution_path.append(currNode.point[1])
                    ax.plot([currNode.point[0], currNode.parent.point[0]],[currNode.point[1], currNode.parent.point[1]], color = 'r', linestyle = '-', linewidth = 2)
                    plt.draw()
                    currNode = currNode.parent
                solution_path.reverse()
                print solution_path
                return solution_path

                
            elif currentState == 'buildTree':
                count += 1
                # Check that we haven't created too many nodes
                if count < self.NUMNODES:
                    foundNext = False
                    # If we haven't found our next point, then generate more random points until it works out.
                    while foundNext == False:
                        ##### Why multiple points? Efficiency? #####
                        rand = random_point()
                        rand2 = random_point()
                        rand3 = random_point()
                        parentNode = nodes[0]
                        # If the distance from the node to the random point is less than that from the parent node
                        # to the random point, then create a new point.
                        ##### This is the * part of RRT*, right? ##### 
                        for p in nodes:
                            if dist(p.point,rand) <= dist(parentNode.point,rand) and dist(p.point,rand2) <= dist(parentNode.point,rand2) and dist(p.point,rand3) <= dist(parentNode.point,rand3):
                                newPoint = step_from_to(p.point,rand)
                                newPoint2 = step_from_to(p.point,rand2)
                                newPoint3 = step_from_to(p.point,rand3)
                                # If the new points are unoccupied (ie white), let point p be the new parent.
                                if self.map_img[int(newPoint[1])][int(newPoint[0])][0] == 255 and self.map_img[int(newPoint2[1])][int(newPoint[0])][0] == 255 and self.map_img[int(newPoint3[1])][int(newPoint[0])][0] == 255:
                                    parentNode = p
                                    foundNext = True

                    # Plot all the nodes and lines connecting them for each different tree in different colors.
                    newnode = step_from_to(parentNode.point,rand)
                    newnode2 = step_from_to(parentNode.point,rand2)
                    newnode3 = step_from_to(parentNode.point,rand3)
                    nodes.append(Node(newnode, parentNode))
                    nodes.append(Node(newnode2, parentNode))
                    nodes.append(Node(newnode3, parentNode))
                    ax.plot([parentNode.point[0], newnode[0]], [parentNode.point[1], newnode[1]], color = 'blue', linestyle = '-', linewidth = 2)
                    plt.draw()
                    ax.plot([parentNode.point[0], newnode[0]], [parentNode.point[1], newnode2[1]], color = 'g', linestyle = '-', linewidth = 2)
                    plt.draw()
                    ax.plot([parentNode.point[0], newnode[0]], [parentNode.point[1], newnode3[1]], color = 'pink', linestyle = '-', linewidth = 2)
                    plt.draw()


                    # If tree collides with goal point, goal has been reached
                    if goal_collision(newnode, goalPoint.point, self.GOAL_RADIUS) or goal_collision(newnode2, goalPoint.point, self.GOAL_RADIUS) or goal_collision(newnode3, goalPoint.point, self.GOAL_RADIUS):
                        currentState = 'goalFound'
                        goalNode = nodes[len(nodes)-1]
                
                # If number of nodes has been exceeded and goal point is not found, stop.
                else:
                    print("Ran out of nodes")
                    return;


    if __name__ == '__main__':
        main()
        
    #    # Tell ROS that we're making a new node.
    #    rospy.init_node("Safety_Node")
    #
    #    # Init the node
    #    Safety()
    #
    #    # Don't let this script exit while ROS is still running
    #    rospy.spin()
