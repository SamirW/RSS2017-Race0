#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32, Float32MultiArray  # the rostopic message we subscribe
from ackermann_msgs.msg import AckermannDriveStamped

class vis_cone_avoidance(object):

    def __init__(self):

        self.sub = rospy.Subscriber("pursuit_control_input",\
            Float32MultiArray, self.updateControl, queue_size=1)

        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_1",\
            AckermannDriveStamped, queue_size=1)

        x = np.array([3, 2, 1, 3, 1, 2, 3, 1, 3, 5, 3, 1, 6, 4, 2, 7])
        y = np.array([106, 140, 224, 105, 230, 143, 109, 222, 106, 69, 110, 220, 55, 83, 152, 49])
        degree = 3
        self.polyfit = np.polyfit(y, x, degree)

        self.Kp = 1.0

        self.control_dist = 4.5

        self.shiftSetPoint = 1.0

        self.speed = 2.0

        self.camwidth = 672.0
        self.camheight = 376.0

    def get_shift_setpoint(self, cone_height, color):
        if not color: 
            self.shiftSetPoint =  -cone_height/self.camheight
            return self.shiftSetPoint

        else:
            self.shiftSetPoint = cone_height/self.camheight
            return shiftSetPoint

    def updateControl(self, cone_msg):

        shift, height, color = cone_msg.data
        dist = max(np.polyval(self.polyfit, height), 0)

        if dist <= control_dist:

            print "avoiding cone"

            setpoint = self.get_shift_setpoint(height, color)

            shiftError = shift - setpoint

            u_s = self.Kp * shiftError

            drive_msg_stamped = AckermannDriveStamped()
            drive_msg = AckermannDrive()
            drive_msg.speed = self.speed
            drive_msg.steering_angle = u_s
            drive_msg.acceleration = 0
            drive_msg.jerk = 0
            drive_msg.steering_angle_velocity = 0
            drive_msg_stamped.drive = drive_msg
            self.control_pub.publish()

            self.pub.publish(drive_msg_stamped)

if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('vis_cone_avoidance')

    # create Echo to start the image passthrough
    c = vis_cone_avoidance()

    # continue running echo until node is killed from outside process
    rospy.spin()
