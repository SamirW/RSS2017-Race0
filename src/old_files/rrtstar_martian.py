#!/usr/bin/python
"""
Created on Sun Apr 11 

@author: samir and martian
"""

import random
import numpy as np
import math
import time
from matplotlib import pyplot as plt
from matplotlib import cm
from scipy.misc import imread
import range_libc

class Node(object):
    def __init__(self, point, parent=None, cost=0):
        self.point = point
        self.parent = parent
        self.cost = cost

class RRT(object):
    def __init__(self, environment, start, goal):
        self.map_loc = environment
        self.start_loc = start
        self.goal_loc = goal
        self.goal_loc_np = np.array(goal)

        # Tune for RRT #
        self.NUMNODES = 10000
        self.max_dist = 20
        self.min_dist = 1
        self.goal_check_period = 100
        self.goal_check_radius = 10
        self.rrt_star_radius = 100

        # Load and store map #
        self.map_img = imread(self.map_loc)
        self.mapWidth = len(self.map_img[0])
        self.mapHeight = len(self.map_img)
        self.open_cells = self.load_map(self.map_img)
        self.open_cells_tuple = None
        self.make_wall()

        # Tracker variables #
        self.max_dist_sq = self.max_dist**2
        self.min_dist_sq = self.min_dist**2
        self.goal_check_radius_sq = self.goal_check_radius**2
        self.rrt_star_radius_sq = self.rrt_star_radius**2
        self.nodes = list()
        self.nodeLocs = np.array(self.start_loc, ndmin=2)
        self.nodeDict = dict()
        self.solutionPath = None
        self.count = 0
        self.pathFound = False
        self.foundGoalNode = None
        self.startTime = None

    def load_map(self, map_img):
        cells = set()

        for i in range(len(map_img[0])):
            for j in range(len(map_img)):
                if map_img[i][j] == 255:
                    cells.add((j,i))

        return cells

    def make_wall(self):
        n_ranges = 2
        theta = np.pi/2.
        y = self.start_loc[0] - 10*np.sin(theta)
        x = self.start_loc[1] - 10*np.cos(theta)
        max_scan_angle = 90*(np.pi/180)
        testMap = range_libc.PyOMap(self.map_loc,1)

        bl = range_libc.PyBresenhamsLine(testMap, 500)
        queries = np.zeros((n_ranges, 3), dtype=np.float32)
        ranges = np.zeros(n_ranges, dtype=np.float32)
        queries[:,0] = x  
        queries[:,1] = y 
        queries[:,2] = theta + np.linspace(-max_scan_angle, max_scan_angle, n_ranges)
        points = bl.calc_range_many(queries,ranges)
        bl.saveTrace("./test.png")
        # print imread("./test.png")[500][600]
        map_img = imread("./test.png")

        cells_to_remove = set()
        for i in range(len(map_img[0])):
            for j in range(len(map_img)):
                if map_img[i][j][2] == 200:
                    self.open_cells.discard((j,i))
                    # print (j,i)

        self.open_cells_tuple = tuple(self.open_cells)

        self.start_flag = True
        print "wall made; set goal!"

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

    def nearestLocs(self, pointLoc):
        '''
        Returns nearest node locs within radius from given loc
        Numpy array
        '''
        dists = self.distSquared(self.nodeLocs, pointLoc)
        indices = np.argwhere(dists < self.rrt_star_radius_sq).flatten()
        return self.nodeLocs[indices]

    def createSolution(self, goalNode):
        self.solutionPath = [goalNode]
        node = goalNode
        while node is not None:
            parentNode = node.parent
            if parentNode is not None:
                self.solutionPath.append(parentNode)
            node = parentNode

        self.solutionPath.reverse()

    def publishSolution(self):
        output = []
        for node in self.solutionPath:
            output.append(tuple(node.point))
        output = np.array(output).flatten()
        output = list(output)
        self.pub_control

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
        # Start clock
        self.startTime = time.time()
        # Initialize start node
        startNode = Node(np.array(self.start_loc))
        self.nodes.append(startNode)
        self.nodeDict[self.start_loc] = startNode

        # Goal flag
        searching = True

        while searching:
            if self.count >= self.NUMNODES:
                searching = False

            #### REMOVE ####
            if self.foundGoalNode:
                searching = False

            if (self.count % 1000 == 0) and self.foundGoalNode:
                searching = False
            
            x_rand = self.randomPoint() # Numpy array
            x_nearest = self.nearest(x_rand) # Numpy array
            x_new, cost = self.steerPoint(x_nearest, x_rand) # Numpy array, and scalar
            if x_new is not None:
                if not self.checkCollision(x_nearest, x_new):
                    nearestLocs = self.nearestLocs(x_new)

                    # RRT star part 1 // Connect along minimum path
                    x_min = x_nearest # Numpy array
                    cost_min = cost
                    for loc in nearestLocs:
                        if not self.checkCollision(loc, x_new):
                            curr_node = self.nodeDict[tuple(loc)]
                            node_cost = curr_node.cost
                            new_cost = node_cost + np.sqrt(self.distSquared(loc, x_new))
                            if new_cost < cost_min:
                                x_min = loc
                                cost_min = new_cost

                    # RRT star part 2 // Wire new node
                    parentNode = self.nodeDict[tuple(x_min)]
                    newNode = Node(x_new, parent=parentNode, cost=parentNode.cost + cost_min)
                    # Add to trackers
                    self.nodes.append(newNode)
                    self.nodeLocs = np.vstack([self.nodeLocs, x_new])
                    self.nodeDict[tuple(x_new)] = newNode

                    # RRT star part 3 // Rewire the tree
                    for loc in nearestLocs:
                        if not self.checkCollision(loc, x_new):
                            curr_node = self.nodeDict[tuple(loc)]
                            new_cost = newNode.cost + np.sqrt(self.distSquared(x_new, loc))
                            if new_cost < curr_node.cost:
                                curr_node.parent = newNode
                                curr_node.cost = new_cost

                    # Goal check
                    if self.goalCheck(x_new):
                        searching = True
                        self.pathFound = True
                        self.foundGoalNode = newNode

            self.count += 1

        if not self.pathFound:
            print "woops"
            self.plotAll()
        else:
            self.createSolution(self.foundGoalNode)
            print time.time() - self.startTime
            print self.count
            self.plotSolution()
        return self.count


if __name__ == '__main__':
    test = RRT('../maps/basement_fixed_more_dilated.png', (476, 327), (402, 317))

    test.findPath()
