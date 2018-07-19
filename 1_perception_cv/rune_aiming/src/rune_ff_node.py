#!/usr/bin/python

'''
This ros package uses the world coordinate result by "solvePnP".
'''
import roslib
import numpy as np
import sys
import rospy
from geometry_msgs.msg import Point, Twist
import math


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


class rune_feedforward:
    def __init__(self):
        self.armor_subscriber = rospy.Subscriber(
            "/rune_locations", Point, self.callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/rune_cmd', Twist, queue_size=1)

        self.last_angular_y = 0.0
        self.last_angular_z = 0.0

    def callback(self, point):
        vel_msg = Twist()
        if abs(point.x) < sys.float_info.epsilon:
            vel_msg.angular.y = self.last_angular_y
            vel_msg.angular.z = self.last_angular_z
        else:
            k_y = 0.0
            k_z = 0.0
            center_x = 0.0
            center_y = 0.0
            if rospy.has_param('/server_node/k_y'):
                k_y = rospy.get_param('/server_node/k_y')
                k_z = rospy.get_param('/server_node/k_z')
                center_x = rospy.get_param('/server_node/center_x')
                center_y = rospy.get_param('/server_node/center_y')
            #theta should be modified for different sentries
            theta =  2*math.pi / 45
            rune_T_camera = np.array(
                [point.x, point.y*np.cos(theta)-point.z*np.sin(theta), point.z*np.cos(theta)+point.y*np.sin(theta)])
            # yangjiao = np.array([[1, 0, 0],
            #                      [0, np.cos(theta), -np.sin(theta)],
            #                      [0, np.sin(theta), np.cos(theta)]])
            # rune_T_camera = yangjiao.dot(rune_T_camera)
            # rospy.loginfo(
            #     "rune center temp: %f, %f, %f", rune_T_camera[0], rune_T_camera[1], rune_T_camera[2])
            opencv_rotation = np.array([[0, 0, 1],
                                        [-1, 0, 0],
                                        [0, -1, 0]])
            rune_T_camera_rot = opencv_rotation.dot(rune_T_camera)
            # to be modified
            # depends on the location of camera
            # old board gimbal, numer one infrantry
            camera_T_gimbal = np.array([130, -110, -250])
            T = rune_T_camera_rot + camera_T_gimbal

            normalized_T = T / np.linalg.norm(T)
            x_axis = np.array([1, 0, 0])
            axis = np.cross(normalized_T, x_axis)
            normalized_axis = axis / np.linalg.norm(axis)
            angle = np.arccos(x_axis.dot(normalized_T))

            R = rotation_matrix(normalized_axis, angle)
            R = np.transpose(R)
            T_euler0 = np.arctan2(R[1, 0], R[0, 0])
            # T_euler1 = np.arccos(R[1,0] / math.sin(T_euler0))
            T_euler1 = np.arcsin(-R[2, 0])
            T_euler2 = np.arctan2(R[2, 1], R[2, 2])

            # rospy.loginfo(
            #     "rune center in gimbal rotation center: %f, %f, %f", T[0], T[1], T[2])
            # rospy.loginfo("rune center euler angle zyx: %f, %f, %f",
            #               T_euler0, T_euler1, T_euler2)

            vel_msg.angular.y = T_euler1
            vel_msg.angular.z = T_euler0
        self.cmd_pub.publish(vel_msg)


if __name__ == "__main__":
    rospy.init_node('rune_feedforward_node')
    ff = rune_feedforward()
    rospy.spin()
