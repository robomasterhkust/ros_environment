#!/usr/bin/python

'''
This ros package uses the world coordinate result by "solvePnP".
'''
import roslib
import sys
import rospy
from geometry_msgs.msg import Twist
from rm_cv.msg import ArmorRecord
from self_aiming.msg import Pid
import numpy as np
import math
from collections import deque



def rotation_matrix(direction, angle):
    sina = math.sin(angle)
    cosa = math.cos(angle)
    # rotation matrix around unit vector
    R = np.diag([cosa, cosa, cosa])
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array([[0.0, -direction[2], direction[1]],
                   [direction[2], 0.0, -direction[0]],
                   [-direction[1], direction[0], 0.0]])
    M = np.identity(4)
    M[:3, :3] = R
    return M


class armor_frame_pid:
    def __init__(self):
        # "/prediction_kf/predict",
        self.armor_subscriber = rospy.Subscriber(
            "/detected_armor", ArmorRecord, self.cv_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.debug_pub = rospy.Publisher('/debug', Pid, queue_size=1)

        self.y_err = 0
        self.z_err = 0
        self.prev_y_err = 0
        self.prev_z_err = 0
        self.y_err_int = 0
        self.z_err_int = 0
        self.k_y = 0
        self.k_z = 0

        self.is_armor_valid = False

        # Watch the control diagram for the naming
        self.uy = 0
        self.uz = 0
        self.y0_y = 0
        self.y0_z = 0
        self.y1_y = 0
        self.y1_z = 0
        self.dp_y = 0
        self.dp_z = 0
        self.ysp_y = 0
        self.ysp_z = 0
        self.yp_y = 0
        self.yp_z = 0


    def cv_callback(self, subArmorRecord):
        if abs(subArmorRecord.armorPose.linear.x) < sys.float_info.epsilon:
            self.is_armor_valid = False
        else:
            self.is_armor_valid = True
            shield_T_camera = np.array(
                [subArmorRecord.armorPose.linear.x, subArmorRecord.armorPose.linear.y, subArmorRecord.armorPose.linear.z])
            opencv_rotation = np.array([[ 0, 0, 1],
                                        [-1, 0, 0],
                                        [ 0,-1, 0]])

            shield_T_camera_rot = opencv_rotation.dot(shield_T_camera)

            camera_T_gimbal = np.array([150, 45, -30])
            T = shield_T_camera_rot + camera_T_gimbal

            normalized_T = T / np.linalg.norm(T)
            x_axis = np.array([1, 0, 0])
            axis = np.cross(normalized_T, x_axis)
            normalized_axis = axis / np.linalg.norm(axis)
            angle = np.arccos(x_axis.dot(normalized_T))

            R = rotation_matrix(normalized_axis, angle)
            R = np.transpose(R)
            T_euler0 = np.arctan2(R[1, 0], R[0, 0])
            T_euler1 = np.arcsin(-R[2, 0])
            T_euler2 = np.arctan2(R[2, 1], R[2, 2])

            rospy.loginfo(
                "armor center in gimbal rotation center: %f, %f, %f", T[0], T[1], T[2])
            rospy.loginfo("armor center euler angle zyx: %f, %f, %f",
                          T_euler0, T_euler1, T_euler2)

            # Observer; y -> O -> y0
            self.y0_y = T_euler1
            self.y0_z = T_euler0

    def shutdown_function(self):
        vel_msg = Twist()
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        self.cmd_pub.publish(vel_msg)
        rospy.loginfo("shutting down armor frame smith predictor node")

    def smith_predictor_with_pid_controller(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            # init controller parameter
            y_kp = 0.0
            y_kd = 0.0
            y_ki = 0.0
            z_kp = 0.0
            z_kd = 0.0
            z_ki = 0.0
            y_err_int_max = 0
            z_err_int_max = 0

            if rospy.has_param('/server_node/y_kp'):
                self.k_y = rospy.get_param('/server_node/k_y')
                self.k_z = rospy.get_param('/server_node/k_z')
                y_kp = rospy.get_param('/server_node/y_kp')
                y_kd = rospy.get_param('/server_node/y_kd')
                y_ki = rospy.get_param('/server_node/y_ki')
                z_kp = rospy.get_param('/server_node/z_kp')
                z_kd = rospy.get_param('/server_node/z_kd')
                z_ki = rospy.get_param('/server_node/z_ki')
                self.ysp_z = rospy.get_param('/server_node/center_x')
                self.ysp_y = rospy.get_param('/server_node/center_y')
                y_err_int_max = rospy.get_param('/server_node/y_err_int_max')
                z_err_int_max = rospy.get_param('/server_node/z_err_int_max')

            # Controller C; e -> C -> u
            self.y_err = self.ysp_y - self.yp_y - self.dp_y
            self.y_err_int += self.y_err
            if(abs(self.y_err_int) > y_err_int_max):
                if (self.y_err_int > 0):
                    self.y_err_int = y_err_int_max
                else:
                    self.y_err_int = -y_err_int_max
            self.uy = y_kp * self.y_err + y_kd * \
                      (self.y_err - self.prev_y_err) + y_ki * self.y_err_int


            self.z_err = self.ysp_z - self.yp_z - self.dp_z
            self.z_err_int += self.z_err
            if(abs(self.z_err_int) > z_err_int_max):
                if (self.z_err_int > 0):
                    self.z_err_int = z_err_int_max
                else:
                    self.z_err_int = -z_err_int_max
            self.uz = z_kp * self.z_err + z_kd * \
                      (self.z_err - self.prev_z_err) + z_ki * self.z_err_int

            self.publish_vel()
            rate.sleep()

            # the frequency domain model Gp; u -> Gp -> Yp
            # TODO: add the model, right now is just 1
            self.yp_y = self.uy
            
            self.yp_z = self.uz
            
            # the delay term Dp; yp -> Dp -> y1
            # Dp = e^-51
            # TODO: think about how to add delay, right now is zero
            self.y1_y = self.yp_y

            self.y1_z = self.yp_z

            # the filter after predicted result
            # F = 5e-5 / (z - 1)
            # TODO: add the memory for the filter
            self.dp_y = self.y0_y - self.y1_y

            self.dp_z = self.y0_z - self.y1_z

    def publish_vel(self):
        vel_msg = Twist()
        if self.is_armor_valid:
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
        else:
            vel_msg.angular.y = self.uy
            vel_msg.angular.z = self.uz
            vel_msg.linear.y = self.y0_y * self.k_y
            vel_msg.linear.z = self.y0_z * self.k_z


if __name__ == "__main__":
    rospy.init_node('smith_predictor_with_pid')
    pid = armor_frame_pid()
    rospy.on_shutdown(pid.shutdown_function)

    try:
        pid.smith_predictor_with_pid_controller()
    except rospy.ROSInterruptException:
        pass
