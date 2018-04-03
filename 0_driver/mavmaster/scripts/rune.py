#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Point

def rune():
	pub = rospy.Publisher("rune_result",Point,queue_size=10)
	rospy.init_node('rune',anonymous=True)
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
			x = random.random()
			y = random.random()
			z = random.random()
			st = "Publishing rune x = {}, y = {}, z = {}".format(x,y,z)
			rospy.loginfo(st)
			p = Point(x,y,z)
			pub.publish(p)
			rate.sleep()

if __name__ == '__main__':
	try:
		rune()
	except rospy.ROSInterruptException:
		pass
