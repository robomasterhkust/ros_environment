#!/usr/bin/env python

import roslib
import numpy as np
import sys
import rospy
import cv2
from geometry_msgs.msg import Point, Twist
import numpy as np


class armor_pid:
    def __init__(self):
        self.result_sub = rospy.Subscriber(
            "/armor_center", Point, self.callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.y_err = 0
        self.z_err = 0
        self.prev_y_err = 0
        self.prev_z_err = 0
        self.image_center_x = rospy.get_param('center_x', 640)
        self.image_center_y = rospy.get_param('center_y', 512)

    def callback(self, point):
        vel_msg = Twist()
        if(abs(point.x) > sys.float_info.epsilon and abs(point.y) > sys.float_info.epsilon):
            self.y_err = -1 + point.y / self.image_center_y
            self.z_err = 1 - point.x / self.image_center_x
            y_kp = rospy.get_param('/server_node/y_kp')
            y_kd = rospy.get_param('/server_node/y_kd')
            z_kp = rospy.get_param('/server_node/z_kp')
            z_kd = rospy.get_param('/server_node/z_kd')
            rospy.loginfo("y_kp: %f,y_kd: %f", y_kp, y_kd)
            vy = y_kp * self.y_err + y_kd * (self.y_err - self.prev_y_err)
            vz = z_kp * self.z_err + z_kd * (self.z_err - self.prev_z_err)
            self.prev_y_err = self.y_err
            self.prev_z_err = self.z_err

            vel_msg.angular.y = vy
            vel_msg.angular.z = vz
            rospy.loginfo("entering armor pid")
        else:
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

        self.cmd_pub.publish(vel_msg)
        rospy.loginfo("vy:%f,vz:%f", vel_msg.angular.y, vel_msg.angular.z)


if __name__ == "__main__":
    rospy.init_node('armor_pid_node')
    pid = armor_pid()
    rospy.spin()
