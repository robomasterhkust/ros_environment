#!/usr/bin/python

'''
This ros package uses the world coordinate result by "solvePnP".
'''
import roslib
import sys
import rospy
from geometry_msgs.msg import Twist
from rm_cv.msg import ArmorRecord
from can_receive_msg.msg import imu_16470
from self_aiming.msg import Pid
import numpy as np
import math
from collections import deque
from tf.transformations import quaternion_multiply, quaternion_matrix

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
        # self.imu_16470_subscriber = rospy.Subscriber(
        #     "/can_receive_node/imu_16470", imu_16470, self.imu_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel_ref', Twist, queue_size=1)
        self.debug_pub = rospy.Publisher('/debug', Pid, queue_size=1)

        self.y_err = 0
        self.z_err = 0
        self.prev_y_err = 0
        self.prev_z_err = 0
        self.y_err_int = 0
        self.z_err_int = 0
        #
        # self.time_queue = deque()
        # self.imu_queue = deque()

    # def imu_callback(self, subImu_16470):
    #     current_quaternion = np.array([
    #         subImu_16470.quaternion[1], subImu_16470.quaternion[2], subImu_16470.quaternion[3], subImu_16470.quaternion[0]])
    #     current_time = subImu_16470.header.stamp
    #     self.imu_queue.append(current_quaternion)
    #     self.time_queue.append(current_time)
    #     rospy.loginfo("imu callback -- queue length: %d", len(self.imu_queue))

    def cv_callback(self, subArmorRecord):
        vel_msg = Twist()
        pid_msg = Pid()
        if abs(subArmorRecord.armorPose.linear.x) < sys.float_info.epsilon:
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
        else:
            # image_time = subArmorRecord.header.stamp
            # # rospy.loginfo("cv time: %s" % (image_time.to_sec()))
            # rospy.loginfo("begin: length %d", len(self.imu_queue))
            # start_index = 1
            # for i in range(len(self.time_queue)):
            #     if self.time_queue[i] < image_time:
            #         continue
            #     else:
            #         start_index = i
            #         break
            #
            # for _ in range(start_index - 1):
            #     # rospy.loginfo("pop left")
            #     self.imu_queue.popleft()
            #     self.time_queue.popleft()
            # rospy.loginfo("after: length %d", len(self.imu_queue))
            #
            # start_quaternion = self.imu_queue[0]
            # print "The starting quaternion is %s %s %s %s." % (
            #     start_quaternion[0], start_quaternion[1], start_quaternion[2], start_quaternion[3])
            #
            # end_quaternion = self.imu_queue[len(self.imu_queue) - 1]
            # print "The ending quaternion is %s %s %s %s." % (
            #     end_quaternion[0], end_quaternion[1], end_quaternion[2], end_quaternion[3])
            #
            # start_quaternion[3] = -start_quaternion[3]
            #
            # diff_quaternion = quaternion_multiply(
            #     end_quaternion, start_quaternion)
            # diff_rotation_matrix = quaternion_matrix(diff_quaternion)
            # diff_rotation_matrix = diff_rotation_matrix[0:3, 0:3]
            # print "The quaternion representation is %s %s %s %s." % (
            #     diff_quaternion[0], diff_quaternion[1], diff_quaternion[2], diff_quaternion[3])
            # print diff_rotation_matrix
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

            shield_T_camera = np.array(
                [subArmorRecord.armorPose.linear.x, subArmorRecord.armorPose.linear.y, subArmorRecord.armorPose.linear.z])
            opencv_rotation = np.array([[0, 0, 1],
                                        [-1, 0, 0],
                                        [0, 1, 0]])

            shield_T_camera_rot = opencv_rotation.dot(shield_T_camera)
            # for soldier 2
            # camera_T_gimbal = np.array([135, 0, 0])
            # for soldier 1
            # camera_T_gimbal = np.array([185, 0, 0])

            camera_T_gimbal = np.array([150, 0, -30])
            T = shield_T_camera_rot + camera_T_gimbal
            # print "before: %s %s %s" % (T[0], T[1], T[2])
            # T = diff_rotation_matrix.dot(T)
            # print "after: %s %s %s" % (T[0], T[1], T[2])

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

            rospy.loginfo(
                "armor center in gimbal rotation center: %f, %f, %f", T[0], T[1], T[2])
            rospy.loginfo("armor center euler angle zyx: %f, %f, %f",
                          T_euler0, T_euler1, T_euler2)

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
            pid_msg.y_d = self.y_err - self.prev_y_err
            pid_msg.y_i = self.y_err_int
            pid_msg.z_p = self.z_err
            pid_msg.z_d = self.z_err - self.prev_z_err
            pid_msg.z_i = self.z_err_int
            vy = y_kp * self.y_err + y_kd * \
                (self.y_err - self.prev_y_err) + y_ki * self.y_err_int
            vz = z_kp * self.z_err + z_kd * \
                (self.z_err - self.prev_z_err) + z_ki * self.z_err_int
            self.prev_y_err = self.y_err
            self.prev_z_err = self.z_err

            vel_msg.angular.y = vy
            vel_msg.angular.z = vz
            vel_msg.linear.y = T_euler1 * k_y
            vel_msg.linear.z = T_euler0 * k_z


            self.debug_pub.publish(pid_msg)
        self.cmd_pub.publish(vel_msg)

    def shutdown_function(self):
        # self.imu_queue.clear()
        # self.time_queue.clear()
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
