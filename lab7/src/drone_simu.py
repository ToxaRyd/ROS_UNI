#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Pose
from std_msgs.msg import Empty
from math import atan2
from pynput.keyboard import Key, Listener

global x
global y
global z
global theta

x = 0.0
y = 0.0
z = 0.0
theta = 0

speed = Twist()

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

take_off = rospy.Publisher("/drone/takeoff", Empty, queue_size = 1)
land = rospy.Publisher("/drone/land", Empty, queue_size = 1)

def on_press(key):
    global speed
    if key == Key.page_up:
        take_off.publish(to_send)
    elif key == Key.page_down:
        land.publish(to_send)

    elif key == Key.up:
        speed.linear.x = 1.0
    elif key == Key.down:
        speed.linear.x = -1.0
    
    elif key == Key.left:
        speed.angular.z = 1.0
    elif key == Key.right:
        speed.angular.z = -1.0
    return False

def on_release(key):
    print("STOPPED!")
    global speed
    if key == Key.up:
        speed.linear.x = 0.0
    elif key == Key.down:
        speed.linear.x = 0.0
    
    elif key == Key.left:
        speed.angular.z = 0.0
    elif key == Key.right:
        speed.angular.z = 0.0
    return False

rospy.init_node("lab7")
r = rospy.Rate(10)

def Callback(msg):
    global x
    global y
    global z
    global theta

    x = msg.position.x
    y = msg.position.y
    y = msg.position.z
    rot_q = msg.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

sub = rospy.Subscriber("/drone/gt_pose", Pose, Callback)
to_send = Empty()

if __name__ == '__main__':
    try:
        print('ready!')
        print('Use page_up and page_down to take off and land;')
        print('Use arrows to move and rotate (press shortly once to add speed, press for a sec and releas to stop!)')
        while not rospy.is_shutdown():
            #print(x, y, z, theta)
            with Listener(
                on_press=on_press,
                on_release=on_release) as listener:
                listener.join()
                
            pub.publish(speed)
            r.sleep()
            #rospy.spin()
    except KeyboardInterrupt:
        pass
