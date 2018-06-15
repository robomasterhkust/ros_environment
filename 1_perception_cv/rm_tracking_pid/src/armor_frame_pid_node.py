#!/usr/bin/python

'''
This ros package uses the world coordinate result by "solvePnP".
'''
import roslib
import numpy as np
import sys
import rospy
import cv2
from geometry_msgs.msg import TwistStamped, Twist
import numpy as np


class armor_frame_pid:
    def __init__(self):
        self.camera_topic = rospy.get_param(
            '/camera_twist', "/gimbal_detected_armor")
        self.armor_subscriber = rospy.Subscriber(
            self.camera_topic, TwistStamped, self.callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.y_err = 0
        self.z_err = 0
        self.prev_y_err = 0
        self.prev_z_err = 0

    def callback(self, pnp):
        vel_msg = Twist()
        shield_T_camera = np.array(
            [pnp.twist.linear.x, pnp.twist.linear.y, pnp.twist.linear.z])
