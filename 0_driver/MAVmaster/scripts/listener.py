#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

def armor_callback(data):
	rospy.loginfo("Turning gimbal to {}, {}, {}".format(data.x,data.y,data.z))

def rune_callback(data):
	rospy.loginfo("Rune Detected @ {}, {}, {}".format(data.x,data.y,data.z))

def listener():
	rospy.init_node("listener",anonymous=True)
	rospy.Subscriber("gimbal_ang_vel",Vector3,armor_callback)
	rospy.Subscriber("rune_result",Point,rune_callback)
	rospy.spin()

if __name__ == '__main__':
	listener();

