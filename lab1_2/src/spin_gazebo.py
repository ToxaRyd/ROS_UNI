#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def spin():
    rospy.init_node('lab1_2')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel = Twist()
    
    vel.linear.x = 0.5
    vel.angular.z = 1.0

    while not rospy.is_shutdown():
        pub.publish(vel)

if __name__ == '__main__':
    try:
        spin()
    except rospy.ROSInterruptException: pass