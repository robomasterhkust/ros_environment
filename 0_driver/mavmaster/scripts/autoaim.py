#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

pub = rospy.Publisher("gimbal_ang_vel",Vector3,queue_size=10)

def autoaim_callback(data):
	rospy.loginfo("Armor @ {}, {}, {}".format(data.x,data.y,data.z))
	x_vel = data.x * 10
	y_vel = data.y * 10
	z_vel = data.z * 10
	ang_vel = Vector3(x_vel,y_vel,z_vel)
	pub.publish(ang_vel)
	
def autoaim():
	rospy.init_node("autoaim",anonymous=True)
	rospy.Subscriber("cv_result",Point,autoaim_callback)
	rospy.spin()

if __name__ == '__main__':
	autoaim()
