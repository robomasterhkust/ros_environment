#!/usr/bin/env python

import roslib
import numpy as np
import sys
import rospy
import cv2
from geometry_msgs.msg import Twist,Point
import numpy as np


class armor_tracking:
    def __init__(self):
        self.armor_center_sub = rospy.Subscriber(
            "/armor_center", Point, self.callback)
        self.result_pub = rospy.Publisher(
            "/armor_center_kf", Twist, queue_size=1)

        self.kf = cv2.KalmanFilter(4, 2)
        cv2.setIdentity(self.kf.transitionMatrix, 1.)
        self.kf.measurementMatrix = np.array([
            [1., 0., 0., 0.],
            [0., 1., 0., 0.]
        ], np.float32)
        cv2.setIdentity(self.kf.measurementNoiseCov, 1e-1)
        self.ticks = cv2.getTickCount()

    def callback(self, point):
        publish_p = Twist()
        if(abs(point.x) > sys.float_info.epsilon and abs(point.y) > sys.float_info.epsilon):
            self.precTick = self.ticks
            self.ticks = cv2.getTickCount()
            dT = (self.ticks - self.precTick) / cv2.getTickFrequency()
            meas = np.array([[point.x], [point.y]], np.float32)
            self.kf.transitionMatrix = np.array([
                [1., 0, dT, 0],
                [0, 1., 0, dT],
                [0, 0, 1., 0],
                [0, 0, 0, 1.]
            ], np.float32)
            vx_cov = rospy.get_param('/server_node/kalman_vx_cov')
            vy_cov = rospy.get_param('/server_node/kalman_vy_cov')
            self.kf.processNoiseCov = np.array([
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, vx_cov, 0],
                [0, 0, 0, vy_cov]
            ], np.float32)
            self.kf.correct(meas)

            state = self.kf.predict()
            
            publish_p.linear.x = state[0]
            publish_p.linear.y = state[1]
            publish_p.linear.z = 0.0
            publish_p.angular.x = state[2]
            publish_p.angular.y = state[3]
            publish_p.angular.z = 0.0
        else:
            publish_p.linear.x = 0.0
            publish_p.linear.y = 0.0
            publish_p.linear.z = 0.0
            publish_p.angular.x = 0.0
            publish_p.angular.y = 0.0
            publish_p.angular.z = 0.0

        self.result_pub.publish(publish_p)


def main(args):
    rospy.init_node('armor_tracking_node', anonymous=True)
    ct = armor_tracking()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
