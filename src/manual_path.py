#!/usr/bin/env python

import utils_TA
from threading import Lock
import tf
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
import numpy as np
import rospy

class ManualPath(object):
    """ 
        Publishes a manual path for the time trial
    """
    def __init__(self):
        # Local trackers/variables
        self.start_lock = Lock()
        self.pathTuples = [(576, 324), (568, 324), (558, 324), (545, 325), (532, 329), \
            (521, 335), (511, 342), (503, 349), (495, 360), (490, 368), (487, 378), \
            (483, 387), (454, 459), (450, 468), (445, 478), (439, 486), (432, 494), \
            (427, 504), (420, 510), (220, 735), (214, 744), (206, 755), (202, 764), \
            (198, 775), (194, 789), (195, 802), (201, 815), (211, 827), (222, 833), \
            (234, 836), (244, 839), (253, 839), (261, 839), (459, 839), (468, 843), \
            (478, 849), (482, 859), (485, 870), (485, 885), (485, 896), (485, 978), \
            (487, 986), (487, 996), (500, 1003), (515, 1007), (526, 1008), (540, 1008), \
            (1107, 1008), (1116, 1008), (1125, 1008), (1136, 1004), (1146, 1000), (1157, 994), \
            (1166, 987), (1171, 973), (1174, 964), (1174, 960), (1174, 388), (1174, 378), (1172, 369), \
            (1165, 353), (1154, 341), (1138, 332), (1123, 326), (1108, 324), (1098, 324), (1088, 324), (586, 324)]

        self.pathTuples.reverse()

        self.pathPolygon = None
        self.trajectory = None
        self.start = None
        self.start_flag = False

        self.start_loc = None

        self.inferred_pose_sub = rospy.Subscriber("/infered_pose_map", PoseStamped, self.set_inferred_pose, queue_size=1)

        # Pub topics
        self.path_pub = rospy.Publisher(rospy.get_param("~path_topic", "/path"), \
            PolygonStamped, queue_size=1)   # Path topic

    def set_inferred_pose(self, msg):
        if not self.start_flag:
            print "Setting inferred pose"
            self.start_lock.acquire()
            self.start = (int(msg.pose.position.x), int(msg.pose.position.y))
            self.start_loc = self.start
            self.nodeLocs = np.array(self.start_loc, ndmin=2)
            self.start_orientation = utils_TA.quaternion_to_angle(msg.pose.orientation) + np.pi/2.0
            self.start_flag = True
            self.publishMap()
            self.start_lock.release()
            print "inferred pose set" 

    def publishMap(self):
        '''
        Takes a mapPath [(x1, y1), (x2, y2), ...] in map coordinates and 
        outputs a PolygonStmped of the points in vesc (world) coordinates
        '''
        poly = PolygonStamped()
        poly.header = utils_TA.make_header("/map")

        scale = 0.0504

        for i in xrange(len(self.pathTuples)):
            p = self.pathTuples[i]
            x = 25.90 - p[0]*scale
            y = -16.50 + p[1]*scale

            pt = Point32()
            pt.x = x
            pt.y = y
            pt.z = -1

            poly.polygon.points.append(pt)

        self.path_pub.publish(poly)
        print poly
        print "traj pubbed"

if __name__=="__main__":
    rospy.init_node("manual_path")
    pf = ManualPath()
    rospy.spin()