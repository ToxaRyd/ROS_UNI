#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):

  if msg.ranges[0] > 0.4: #Here I had a problem when tried to put 10 cm distance to an obstacle, it just ignored it and hit the wall
      move.linear.x = 0.4 #When I put speed higher than 0.4 it enables dron rotation so it runs not straight but also with angle speed
      move.angular.z = 0.0

  else:
      move.linear.x = 0.0
      move.angular.z = 0.5

  pub.publish(move)

rospy.init_node('lab3')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
move = Twist()

rospy.spin()
