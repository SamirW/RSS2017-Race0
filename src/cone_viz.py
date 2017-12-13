import rospy
import numpy as np
import time
import utils_TA

from visualization_msgs.msg import Marker



self.odom_sub = rospy.Subscriber(self.odom_topic,  Odometry, self.odom_callback, queue_size=1)
print "Initialized. Waiting on messages..."

# set up the visualization topic to show the cones that have been detected
self.viz_namespace = "/cone_detect"
self.cone_pub = rospy.Publisher(self.viz_namespace + "/found_cone", Marker, queue_size = 1)
#self.green_cone_pub = rospy.Publisher(self.viz_namespace + "/green_cone", Marker, queue_size = 1)

cone_type = cone_color
cone_pos_x = worldx
cone_pos_y = worldy

car_pos_x = ParticleFilter()[0]
car_pos_y = ParticleFilter()[1]

cone_pos_map_x = car_pos_x + cone_pos_x
cone_pos_map_y = car_pos_y + cone_pos_y

if cone_type == 0:
  x_avoid_shift = -1.0
  self.cone_point = [cone_pos_map_x, cone_pos_map_y]
  color = [1.0, 0, 0]
elif cone_type == 1:
  x_avoid_shift = 1.0
  self.cone_point = [cone_pos_x, cone_pos_y]
  color = [0, 1.0, 0]
self.cone_pub.publish(utils_TA.make_circle_marker(self.cone_point, 0.5, color, "/map", self.viz_namespace, 0, 5))
