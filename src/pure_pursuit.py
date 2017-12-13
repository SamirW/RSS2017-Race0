#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils_TA

from geometry_msgs.msg import PolygonStamped, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
        Set point determined with the method described here: 
            http://www.ri.cmu.edu/pub_files/pub4/howard_thomas_2006_1/howard_thomas_2006_1.pdf
        Relies on localization for ground truth vehicle position.
    """
    def __init__(self):
        # self.trajectory_topic = #rospy.get_param("~trajectory_topic")
        self.odom_topic       = "/pf/pose/odom" #"/vesc/odom" #rospy.get_param("~odom_topic")
        self.lookahead        = 1.5#rospy.get_param("~lookahead")
        self.min_look         = 1.0
        self.max_look         = 2.5
        self.max_reacquire    = 10.0#rospy.get_param("~max_reacquire")
        self.max_speed        = 0.75 #float(rospy.get_param("~speed"))
        self.min_speed        = 0.75
        self.speed            = 0.75
        self.theta_min        = 2.5 # Min angle error to reduce speeds
        self.theta_max        = 17.5
        self.gain             = (self.max_speed - self.min_speed)/float(self.theta_max - self.theta_min) # Determine speed
        self.wrap             = bool(0)#bool(rospy.get_param("~wrap"))
        wheelbase_length      = 0.25#float(rospy.get_param("~wheelbase"))
        self.drive_topic      = "/vesc/high_level/ackermann_cmd_mux/input/nav_0" #rospy.get_param("~drive_topic")
        self.max_waypoint_dist  = 7.0

        self.trajectory  = utils_TA.LineTrajectory("/followed_trajectory")
        self.model       = utils_TA.AckermannModel(wheelbase_length)
        self.do_viz      = True
        self.odom_timer  = utils_TA.Timer(10)
        self.iters       = 0
        
        self.nearest_point   = None
        self.lookahead_point = None
        self.avoid_dist = float(rospy.get_param("avoid_dist"))
        self.return_to_path_dist = 0.4
        self.waypoints = None

        self.cone_detected    = False
        self.pose             = None
        self.cone_pose        = None
        self.cone_type        = None
        self.new_cone         = False
        self.cone_loc         = None
        self.drive            = True
        self.waypoint_counter = 0

        # set up the visualization topic to show the nearest point on the trajectory, and the lookahead point
        self.viz_namespace = "/pure_pursuit"
        self.nearest_point_pub = rospy.Publisher(self.viz_namespace + "/nearest_point", Marker, queue_size = 1)
        self.lookahead_point_pub = rospy.Publisher(self.viz_namespace + "/lookahead_point", Marker, queue_size = 1)
        self.cone_waypoint_pub = rospy.Publisher(self.viz_namespace + '/cone_waypoint', Marker, queue_size=1)
        self.lookahead_pub = rospy.Publisher(self.viz_namespace + '/cone_lookahead', Marker, queue_size=1)
        
        # topic to send drive commands to
        self.control_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size =1 )

        # topic to listen for trajectories
        self.traj_sub = rospy.Subscriber(rospy.get_param("~path_topic", "/path"), PolygonStamped, \
            self.trajectory_callback, queue_size=1)

        self.cone_sub = rospy.Subscriber("/cone_location", Float32MultiArray, self.cone_cb2, queue_size=1)
        
        # topic to listen for odometry messages, either from particle filter or the simulator
        # self.odom_sub = rospy.Subscriber(self.odom_topic,  Odometry, self.odom_callback, queue_size=1)

        self.localizer_sub = rospy.Subscriber("/pf/viz/inferred_pose", PoseStamped, self.localizer_cb, queue_size = 1)
        print "Initialized. Waiting on messages..."

    def cone_cb(self, msg):
        self.cone_detected = True
        self.new_cone = True
        self.cone_pose = (msg.data[0], msg.data[1])
        self.cone_type = msg.data[2] # 1:green 0:orange
        # print "got cone location", self.cone_pose

    def cone_cb2(self, msg):
        self.cone_detected = True
        self.new_cone = True
        self.waypoints = [(msg.data[0], msg.data[1]),(msg.data[2], msg.data[3])]

    def cone_within_bounds(self, world_loc):
        map_loc = self.w2mRotation * world_loc + self.w2mTranslation # Convert to map coordinates
        map_loc = tuple(np.floor(map_loc))  # Floor and convert to tuple for hasing
        return map_loc in self.acceptable_cone_coordinates

    def avoid_cone2(self, pose):
        waypoint = np.array([self.waypoints[self.waypoint_counter][0], self.waypoints[self.waypoint_counter][1]])
        car_xy = np.array([pose[0], pose[1]])
        car2waypoint = waypoint - car_xy
        dist = np.linalg.norm(car2waypoint)
        theta = np.arctan2(car2waypoint[1], car2waypoint[0])
        car_vec = np.array([np.cos(pose[2]), np.sin(pose[2])])
        theta_dif = np.arccos(np.dot(car_vec, car2waypoint)/(np.linalg.norm(car_vec)*np.linalg.norm(car2waypoint)))
        # color = [0.0, 0.0, 1.0]
        self.lookahead_point = np.array([self.lookahead*np.cos(theta), self.lookahead*np.sin(theta)]) + np.array([pose[0], pose[1]])
        # self.lookahead_pub.publish(utils_TA.make_circle_marker(self.lookahead_point, 0.5, color, "/map", self.viz_namespace, 0, 5))

        if dist > self.return_to_path_dist and theta_dif < np.pi/2 and dist <= self.max_waypoint_dist:
            steering_angle = self.determine_steering_angle(pose, self.lookahead_point)
            self.apply_control(self.speed, steering_angle)

        elif dist <= self.return_to_path_dist: 
           # "cone avoided"
            self.waypoint_counter += 1

        elif self.waypoint_counter == 2:
            self.cone_detected = False
        
        self.visualize()



    def avoid_cone(self, pose):

       # print "avoiding cone"        

        cone_point = np.array([self.cone_pose[0], self.cone_pose[1]])
        car_xy = np.array([pose[0], pose[1]])
        car2cone = cone_point - car_xy
        d = np.linalg.norm(car2cone)

        if self.new_cone and d > .25: 
            if self.cone_type: # Green cone
                direction = -1
            else: # Orange cone
                direction = 1
            r = np.sqrt(d**2 + 1)
            beta = direction*np.arctan2(self.avoid_dist, d) + np.arctan2(car2cone[1], car2cone[0])
            self.cone_loc = car_xy + np.array([r*np.cos(beta), r*np.sin(beta)])
            self.new_cone = False

        color = [0, 0, 1.0]
        self.cone_waypoint_pub.publish(utils_TA.make_circle_marker(self.cone_loc, 0.5, color, "/map", self.viz_namespace, 0, 5))

        delta = self.cone_loc - car_xy

        theta = np.arctan2(delta[1], delta[0])

        car_vec = np.array([np.cos(pose[2]), np.sin(pose[2])])
        theta_dif = np.arccos(np.dot(car_vec, delta)/(np.linalg.norm(car_vec)*np.linalg.norm(delta)))

        print theta_dif*(180/np.pi), " theta_dif"
        # print "theta", theta*(180/np.pi)

        self.lookahead_point = np.array([self.lookahead*np.cos(theta), self.lookahead*np.sin(theta)]) + np.array([pose[0], pose[1]])
        self.lookahead_pub.publish(utils_TA.make_circle_marker(self.lookahead_point, 0.5, color, "/map", self.viz_namespace, 0, 5))

        dist = np.linalg.norm(delta)
       # print "distance", dist

        if dist > self.return_to_path_dist and theta_dif < np.pi/2 and dist <= self.max_waypoint_dist:
            steering_angle = self.determine_steering_angle(pose, self.lookahead_point)
            self.apply_control(self.speed, steering_angle)

        else: 
           # "cone avoided"
            self.cone_detected = False
        
        self.visualize()

    def visualize(self):
        ''' Publishes visualization topics:
               - Circle to indicate the nearest point along the trajectory
               - Circle to indicate the chosen lookahead point
        '''
        if not self.do_viz:
            return
        # visualize: pure pursuit circle, lookahead intersection, lookahead radius line, nearest point
        if self.nearest_point_pub.get_num_connections() > 0 and isinstance(self.nearest_point, np.ndarray):
            self.nearest_point_pub.publish(utils_TA.make_circle_marker(
                self.nearest_point, 0.5, [0.0,0.0,1.0], "/map", self.viz_namespace, 0, 3))

        if self.lookahead_point_pub.get_num_connections() > 0 and isinstance(self.lookahead_point, np.ndarray):
            self.lookahead_point_pub.publish(utils_TA.make_circle_marker(
                self.lookahead_point, 0.5, [1.0,1.0,1.0], "/map", self.viz_namespace, 1, 3))

    def trajectory_callback(self, msg):
    # def traj_sub(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.polygon.points), "points" 
        self.trajectory.clear()
        self.trajectory.fromPolygon(msg.polygon)
        self.trajectory.publish_viz(duration=0.0)

    def odom_callback(self, msg):
        ''' Extracts robot state information from the message, and executes pure pursuit control.
        '''
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, utils_TA.quaternion_to_angle(msg.pose.pose.orientation)])
        self.pure_pursuit(pose)
        
        # this is for timing info
        self.odom_timer.tick()
        self.iters += 1
        if self.iters % 20 == 0:
            print "Control fps:", self.odom_timer.fps()

    def localizer_cb(self, msg):
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, utils_TA.quaternion_to_angle(msg.pose.orientation)])

        if self.cone_detected and self.drive:
            self.avoid_cone2(self.pose)
        elif self.drive:
            self.pure_pursuit(self.pose)
           # print "Pure Pursuit"

        self.iters += 1
        if self.iters % 20 == 0:
           # print "control"
            pass

    def pure_pursuit(self, pose):
        ''' Determines and applies Pure Pursuit control law
            1. Find the nearest point on the trajectory
            2. Traverse the trajectory looking for the nearest point that is the lookahead distance away from the 
               car, and further along the path than the nearest point from step (1). This is the lookahead point.
            3. Determine steering angle necessary to travel to the lookahead point from step (2)
            4. Send the desired speed and steering angle commands to the robot
            Special cases:
                - If nearest_point is beyond the max path reacquisition distance, stop
                - If nearest_point is between max reacquisition dist and lookahead dist, navigate to nearest_point
                - If nearest_point is less than the lookahead distance, find the lookahead point as normal
        '''
        # stop if no trajectory has been received
        if self.trajectory.empty():
            return self.stop()

        # this instructs the trajectory to convert the list of waypoints into a numpy matrix
        if self.trajectory.dirty():
            self.trajectory.make_np_array()

        # step 1
        nearest_point, nearest_dist, t, i = utils_TA.nearest_point_on_trajectory(pose[:2], self.trajectory.np_points)
        self.nearest_point = nearest_point

        if nearest_dist < self.lookahead:
            # step 2
            lookahead_point, i2, t2 = \
                utils_TA.first_point_on_trajectory_intersecting_circle(pose[:2], self.lookahead, self.trajectory.np_points, i+t, wrap=self.wrap)
            if i2 == None:
                if self.iters % 5 == 0:
                    print "Could not find intersection, end of path?"
                self.lookahead_point = None
            else:
                if self.iters % 5 == 0:
                    print "found lookahead point"
                self.lookahead_point = lookahead_point
        elif nearest_dist < self.max_reacquire:
            if self.iters % 5 == 0:
                print "Reacquiring trajectory"
            self.lookahead_point = self.nearest_point
        else:
            self.lookahead_point = None

        # stop of there is no navigation target, otherwise use ackermann geometry to navigate there
        if not isinstance(self.lookahead_point, np.ndarray):
            self.stop()
        else:
            steering_angle = self.determine_steering_angle(pose, self.lookahead_point)
            self.speed = self.determine_speed(steering_angle)
            # send the control commands
            self.apply_control(self.speed, steering_angle)

        self.visualize()

    def determine_steering_angle(self, pose, lookahead_point):
        ''' Given a robot pose, and a lookahead point, determine the open loop control 
            necessary to navigate to that lookahead point. Uses Ackermann steering geometry.
        '''
        # get the lookahead point in the coordinate frame of the car
        rot = utils_TA.rotation_matrix(-pose[2])
        delta = np.array([lookahead_point - pose[0:2]]).transpose()
        local_delta = (rot*delta).transpose()
        local_delta = np.array([local_delta[0,0], local_delta[0,1]])
        # use the ackermann model
        steering_angle = self.model.steering_angle(local_delta)
        return steering_angle

    def determine_speed(self, steering_angle):
        theta_error = np.abs(steering_angle)*(180/np.pi)
        if theta_error > self.theta_min:
            decrease_speed = self.gain * (theta_error - self.theta_min)
        else:
            decrease_speed = 0

        speed = max(self.max_speed - decrease_speed, self.min_speed)
        # self.lookahead = float(self.max_look-self.min_look)/(self.max_speed-self.min_speed) * (speed - self.max_speed) + self.max_look
        # print "Lookahead is ", self.lookahead
        return speed
        
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
        self.control_pub.publish(drive_msg_stamped)
       # print "control applied!"

    def stop(self):
        # print "Stopping"
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        drive_msg.speed = 0
        drive_msg.steering_angle = 0
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
        drive_msg_stamped.drive = drive_msg
        self.control_pub.publish(drive_msg_stamped)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
