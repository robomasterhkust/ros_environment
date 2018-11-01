#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (-0.3, 0, 0, 0),
    ',': (0.3, 0, 0, 0),
    'j': (0, 0.3, 0, 0),
    'l': (0, -0.3, 0, 0),
    'I': (0, 0, -0.1, 0),
    '<': (0, 0, 0.1, 0),
    'J': (0, 0, 0, 0.1),
    'L': (0, 0, 0, -0.1)
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 1.0)
    turn = rospy.get_param("~turn", 1.0)
    vy = 0
    vz = 0
    py = 0
    pz = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                vy = moveBindings[key][0]
                vz = moveBindings[key][1]
                py = moveBindings[key][2]
                pz = moveBindings[key][3]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                vy = 0
                vz = 0
                py = 0
                pz = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = py
            twist.linear.y = pz
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = vy
            twist.angular.z = vz
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
