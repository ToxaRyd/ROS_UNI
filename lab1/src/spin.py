#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def spin():
    rospy.init_node('turtlesim_1', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel = Twist()
    
    vel.linear.x = 2
    vel.angular.z = 1.8

    while not rospy.is_shutdown():
        pub.publish(vel)


if __name__ == '__main__':
    try:
        spin()
    except rospy.ROSInterruptException: pass