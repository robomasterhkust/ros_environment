#!/usr/bin/python

'''
This ros package uses the local frame translation and velocity
from the constant velocity model from Kalman filter
'''
import roslib
import sys
import rospy
from geometry_msgs.msg import Twist
from rm_cv.msg import ArmorRecord
from self_aiming.msg import Pid
import numpy as np
import math

# ros quaternion (x,y,z,w)


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
        # "/detected_armor"
        self.armor_subscriber = rospy.Subscriber(
            "/detected_armor", ArmorRecord, self.cv_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel_origin', Twist, queue_size=1)
        self.debug_pub = rospy.Publisher('/debug', Pid, queue_size=1)

        self.y_err = 0
        self.z_err = 0
        self.y_vel_err = 0
        self.z_vel_err = 0
        self.prev_y_err = 0
        self.prev_z_err = 0
        self.y_err_int = 0
        self.z_err_int = 0

    def cv_callback(self, subArmorRecord):
        vel_msg = Twist()
        pid_msg = Pid()
        if abs(subArmorRecord.armorPose.linear.x) < sys.float_info.epsilon:
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
        else:
            y_kp = 0.0
            y_kd = 0.0
            y_ki = 0.0
            z_kp = 0.0
            z_kd = 0.0
            z_ki = 0.0
            y_err_int_max = 0
            z_err_int_max = 0
            image_center_x = 0
            image_center_y = 0
            k_y = 0.0
            k_z = 0.0
            if rospy.has_param('/server_node/y_kp'):
                k_y = rospy.get_param('/server_node/k_y')
                k_z = rospy.get_param('/server_node/k_z')
                y_kp = rospy.get_param('/server_node/y_kp')
                y_kd = rospy.get_param('/server_node/y_kd')
                y_ki = rospy.get_param('/server_node/y_ki')
                z_kp = rospy.get_param('/server_node/z_kp')
                z_kd = rospy.get_param('/server_node/z_kd')
                z_ki = rospy.get_param('/server_node/z_ki')
                image_center_x = rospy.get_param('/server_node/center_x')
                image_center_y = rospy.get_param('/server_node/center_y')
                y_err_int_max = rospy.get_param('/server_node/y_err_int_max')
                z_err_int_max = rospy.get_param('/server_node/z_err_int_max')

            # Calculate the relative angle
            shield_T_camera = np.array(
                [subArmorRecord.armorPose.linear.x, subArmorRecord.armorPose.linear.y, subArmorRecord.armorPose.linear.z])
            opencv_rotation = np.array([[ 0, 0, 1],
                                        [-1, 0, 0],
                                        [ 0,-1, 0]])

            shield_T_camera_rot = opencv_rotation.dot(shield_T_camera)

            camera_T_gimbal = np.array([130, 0, -40])
            # T = shield_T_camera_rot + camera_T_gimbal
            T = shield_T_camera
            T_abs = np.linalg.norm(T)

            normalized_T = T / T_abs
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

            # Calculate the relative angular velocity
            shield_vel_camera = np.array(
                [subArmorRecord.armorPose.angular.x, subArmorRecord.armorPose.angular.y, subArmorRecord.armorPose.angular.z])
            # vel = opencv_rotation.dot(shield_vel_camera)
            # omega = vel / T_abs # small angle approximation
            omega = shield_vel_camera / T_abs
            self.y_vel_err = omega[1]
            self.z_vel_err = omega[0]

            self.y_err = T_euler1 - image_center_y
            self.z_err = T_euler0 - image_center_x
            print 'y_p:%s' % (self.y_err)
            print 'z_p:%s' % (self.z_err)
            self.y_err_int += self.y_err
            self.z_err_int += self.z_err
            if(abs(self.y_err_int) > y_err_int_max):
                if (self.y_err_int > 0):
                    self.y_err_int = y_err_int_max
                else:
                    self.y_err_int = -y_err_int_max
            if(abs(self.z_err_int) > z_err_int_max):
                if (self.z_err_int > 0):
                    self.z_err_int = z_err_int_max
                else:
                    self.z_err_int = -z_err_int_max
            print 'y_i:%s' % (self.y_err_int)
            print 'z_i:%s' % (self.z_err_int)

            print 'y_d:%s' % (self.y_err - self.prev_y_err)
            print 'z_d:%s' % (self.z_err - self.prev_z_err)

            pid_msg.header.stamp = rospy.get_rostime()
            pid_msg.y_p = self.y_err
            pid_msg.y_d = self.y_vel_err
            pid_msg.y_i = self.y_err_int
            pid_msg.z_p = self.z_err
            pid_msg.z_d = self.z_vel_err
            pid_msg.z_i = self.z_err_int
            vy = y_kp * self.y_err + y_kd * self.y_vel_err + y_ki * self.y_err_int
            vz = z_kp * self.z_err + z_kd * self.z_vel_err + z_ki * self.z_err_int
            # self.prev_y_err = self.y_err
            # self.prev_z_err = self.z_err

            vel_msg.angular.y = vy
            vel_msg.angular.z = vz
            vel_msg.linear.y = T_euler1 * k_y
            vel_msg.linear.z = T_euler0 * k_z


            self.debug_pub.publish(pid_msg)
        self.cmd_pub.publish(vel_msg)

    def shutdown_function(self):
        vel_msg = Twist()
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        self.cmd_pub.publish(vel_msg)
        rospy.loginfo("shutting down armor frame pid node")


if __name__ == "__main__":
    rospy.init_node('armor_frame_pid_node')
    pid = armor_frame_pid()
    rospy.on_shutdown(pid.shutdown_function)
    rospy.spin()
