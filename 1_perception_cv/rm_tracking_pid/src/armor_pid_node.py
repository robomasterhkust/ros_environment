#!/usr/bin/env python

import roslib
import numpy as np
import sys
import rospy
import cv2
from geometry_msgs.msg import Twist
import numpy as np


class armor_pid:
    def __init__(self):
        self.result_sub = rospy.Subscriber(
            "/armor_center_kf", Twist, self.callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.y_err = 0
        self.z_err = 0
        self.prev_y_err = 0
        self.prev_z_err = 0

    def callback(self, point):
        vel_msg = Twist()
        if(abs(point.linear.x) > sys.float_info.epsilon and abs(point.linear.y) > sys.float_info.epsilon):
            image_center_x = rospy.get_param('/server_node/center_x')
            image_center_y = rospy.get_param('/server_node/center_y')
            self.y_err = (-1 + point.linear.y / image_center_y)
            self.z_err = 1 - point.linear.x / image_center_x
            y_kp = rospy.get_param('/server_node/y_kp')
            y_kd = rospy.get_param('/server_node/y_kd')
            z_kp = rospy.get_param('/server_node/z_kp')
            z_kd = rospy.get_param('/server_node/z_kd')
            vy_kp = rospy.get_param('/server_node/vy_kp')
            vz_kp = rospy.get_param('/server_node/vz_kp')
            rospy.loginfo("y_kp: %f,y_kd: %f", y_kp, y_kd)
            vy = y_kp * self.y_err + y_kd * (self.y_err - self.prev_y_err) + vy_kp*point.angular.y
            vz = z_kp * self.z_err + z_kd * (self.z_err - self.prev_z_err) + vz_kp*point.angular.x
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
