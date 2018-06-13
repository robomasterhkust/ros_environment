#!/usr/bin/env python

import roslib
import numpy as np
import sys
import rospy
import cv2
from geometry_msgs.msg import Point, Twist
import numpy as np
from dynamic_reconfigure.server import Server
from rm_tracking_pid.cfg import tuningConfig

y_kp = 0.0
y_kd = 0.0
z_kp = 0.0
z_kd = 0.0


class armor_pid:
    def __init__(self):
        self.result_sub = rospy.Subscriber(
            "/armor_center_kf", Point, self.callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.y_err = 0
        self.z_err = 0
        self.prev_y_err = 0
        self.prev_z_err = 0
        self.image_center_x = rospy.get_param('center_x', 640)
        self.image_center_y = rospy.get_param('center_y', 512)

    def calc(self):
        vy = y_kp * self.y_err + y_kd * (self.y_err - self.prev_y_err)
        vz = z_kp * self.z_err + z_kd * (self.z_err - self.prev_z_err)

        self.prev_y_err = self.y_err
        self.prev_z_err = self.z_err

        vel_msg = Twist()
        vel_msg.angular.y = vy
        vel_msg.angular.z = vz
        self.cmd_pub.publish(vel_msg)

    def callback(self, point):
        self.y_err = self.image_center_x - point.x
        self.z_err = self.image_center_y - point.y
        self.calc()


def callback(config, level):
    y_kp = config.y_kp
    y_kd = config.y_kd
    z_kp = config.z_kp
    z_kd = config.z_kd
    rospy.loginfo("setting y_kp:%f, y_kd:%f, z_kp:%f, z_kd:%f",
                  y_kp, y_kd, z_kp, z_kd)
    return config


if __name__ == "__main__":
    rospy.init_node('armor_pid_node')
    pid = armor_pid()
    srv = Server(tuningConfig, callback)
    rospy.spin()
