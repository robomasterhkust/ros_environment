#!/usr/bin/env python

import roslib
import numpy as np
import sys
import rospy
import cv2
from geometry_msgs.msg import Point
import numpy as np


class armor_tracking:
    def __init__(self):
        self.armor_center_sub = rospy.Subscriber(
            "/armor_center", Point, self.callback)
        self.result_pub = rospy.Publisher(
            "/armor_center_kf", Point, queue_size=1)

        self.kf = cv2.KalmanFilter(4, 2)
        cv2.setIdentity(self.kf.transitionMatrix, 1.)
        self.kf.measurementMatrix = np.array([
            [1., 0., 0., 0.],
            [0., 1., 0., 0.]
        ], np.float32)
        self.kf.processNoiseCov = np.array([
            [1e-3, 0, 0, 0],
            [0, 1e-3, 0, 0],
            [0, 0, 5., 0],
            [0, 0, 0, 5.]
        ], np.float32)
        cv2.setIdentity(self.kf.measurementNoiseCov, 1e-1)
        self.ticks = cv2.getTickCount()

    def callback(self, point=None):
        if point is not None:
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
            self.kf.correct(meas)

            state = self.kf.predict()
            publish_p = Point()
            publish_p.x = state[0]
            publish_p.y = state[1]
            publish_p.z = 0.0
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
