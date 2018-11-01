#!/usr/bin/python

'''
This ros package is created for rune debug purpose.
'''
import roslib
import numpy as np
import sys
import rospy
from geometry_msgs.msg import Twist
import math
import random

three_list = [-0.3, 0, 0.3]


def rune_debug():
    rune_cmd_pub = rospy.Publisher('/rune_cmd', Twist, queue_size=1)
    rospy.init_node('rune_debug')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        random_int_y = random.randint(0, 2)
        random_int_z = random.randint(0, 2)
        twist_msg = Twist()
        twist_msg.angular.y = three_list[random_int_y]
        twist_msg.angular.z = three_list[random_int_z]
        rune_cmd_pub.publish(twist_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        rune_debug()
    except rospy.ROSInterruptException:
        pass
