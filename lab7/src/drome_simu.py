#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

global x
global y
global z
global theta

x = 0.0
y = 0.0
z = 0.0
theta = 0

speed = Twist()

rospy.init_node("lab7")
r = rospy.Rate(10)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

def Callback(msg):
    global x
    global y
    global z
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


goal = Point()
goal.x = 3
goal.y = 3
#goal.x = float(input("X: "))
#goal.y = float(input("Y: "))

sub = rospy.Subscriber("/odom", Odometry, Callback)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            print(x, y, theta)

            angle_to_goal = atan2(goal.y - y, goal.x - x)

            if abs(angle_to_goal - theta) >= 0.2:
                speed.linear.x = 0.0
                speed.linear.z = -0.3
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.3
                speed.linear.z = 0.3
                speed.angular.z = 0.0   
            pub.publish(speed)
            r.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass
