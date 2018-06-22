#!/usr/bin/python

'''
This ros package uses the world coordinate result by "solvePnP".
'''
import roslib
import numpy as np
import sys
import rospy
from geometry_msgs.msg import TwistStamped, Twist
import numpy as np
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


class armor_frame_feedforward:
    def __init__(self):
        self.armor_subscriber = rospy.Subscriber(
            "/beckThing", TwistStamped, self.callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def callback(self, pnp):
        vel_msg = Twist()
        if abs(pnp.twist.linear.x) < sys.float_info.epsilon:
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        else:
            image_center_y = 360
            image_center_x = 640
            y_limit = 0.1
            z_limit = 0.1
            if rospy.has_param('/server_node/image_center_x'):
                image_center_y = rospy.get_param('/server_node/image_center_y')
                image_center_x = rospy.get_param('/server_node/image_center_x')
                y_limit = rospy.get_param('/server_node/y_limit')
                z_limit = rospy.get_param('/server_node/z_limit')
            shield_T_camera = np.array(
                [pnp.twist.linear.x, pnp.twist.linear.y, pnp.twist.linear.z])
            opencv_rotation = np.array([[0, 0, 1],
                                        [-1, 0, 0],
                                        [0, -1, 0]])
            shield_T_camera_rot = opencv_rotation.dot(shield_T_camera)
            camera_T_gimbal = np.array([135, 0, 0])
            T = shield_T_camera_rot + camera_T_gimbal

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

            rospy.loginfo("armor center in gimbal rotation center: %f, %f, %f", T[0], T[1], T[2])
            rospy.loginfo("armor center euler angle zyx: %f, %f, %f", T_euler0, T_euler1, T_euler2)

            image_center_2d = np.array([image_center_x, image_center_y, 1])
            hehe_temp = np.array([[2.0092461474223967e+03, 0., 5.5050445651906716e+02],
                                  [0., 2.0092461474223967e+03, 4.3204184700626911e+02],
                                  [0., 0., 1.]])
            inverse = np.linalg.inv(hehe_temp)
            image_center_in_camera_frame = inverse.dot(image_center_2d)
            I = image_center_in_camera_frame + camera_T_gimbal

            normalized_I = I / np.linalg.norm(I)
            axis = np.cross(normalized_I, x_axis)
            normalized_axis = axis / np.linalg.norm(axis)
            angle = np.arccos(x_axis.dot(normalized_I))

            R = rotation_matrix(normalized_axis, angle)
            I_euler0 = np.arctan2(R[1, 0], R[0, 0])
            I_euler1 = np.arccos(R[1, 0] / np.sin(I_euler0))
            I_euler2 = np.arctan2(R[2, 1], R[2, 2])

            rospy.loginfo("image center in gimbal rotation center:%f, %f, %f", I[0], I[1], I[2])
            rospy.loginfo("image center euler angle zyx: %f, %f, %f", I_euler0, I_euler1, I_euler2)

            vel_msg.angular.y = T_euler1 * y_limit
            vel_msg.angular.z = T_euler0 * z_limit
        self.cmd_pub.publish(vel_msg)


if __name__ == "__main__":
    rospy.init_node('armor_frame_pid_node')
    ff = armor_frame_feedforward()
    rospy.spin()
