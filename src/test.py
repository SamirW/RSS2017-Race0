#!/usr/bin/env python

# packages
import rospy
import numpy as np
import range_libc
import time
from threading import Lock
import tf.transformations
import tf
import utils as Utils
import utils_TA

# messages
from std_msgs.msg import String, Header, Float32MultiArray
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry






class Test():
    def __init__(self):
        self.pub_control = rospy.Publisher("pursuit_control_input",Float32MultiArray, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/vesc/odom", Odometry, self.pub, queue_size=1)
        self.counter = 0
        self.pubbed = False

    def pub(self, msg):
        if self.counter%500 == 0 and not self.pubbed:
            self.pubbed = True
            controlMsg = Float32MultiArray()
            controlMsg.data = [0.0, 5.0, 1]
            self.pub_control.publish(controlMsg)
            print "pubbed"
        self.counter+=1


if __name__ == '__main__':
    rospy.init_node("particle_filter")
    test = Test()
    rospy.spin()