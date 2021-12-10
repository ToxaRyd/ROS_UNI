#! /usr/bin/env python

import rospy, time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

global x, y, z, theta

x = 0.0
y = 0.0
z = 0.0

theta = 0.0

#PID Variables
global kz, ky
global p, d, Kp, Kd
global previous_pos_z, previous_pos_x

p = 0.0
d = 0.0

kz = 1.0
ky = 1.0

previous_pos_z = 0.0
previous_pos_x = 0.0
#Coefficients for controlling the tolerance (My description :D)
Kp = 0.2
Kd = 0.2
#*************

speed = Twist()
to_send = Empty()

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
take_off = rospy.Publisher("/drone/takeoff", Empty, queue_size = 1)
land = rospy.Publisher("/drone/land", Empty, queue_size = 1)

rospy.init_node("lab10_drone")
r = rospy.Rate(10)

#PID Function
def pid_controller(error, first, second):

    p = error*Kp
    d = ((first - second)/10)*Kd
    PID = p + d

    return PID
#************

#Reading Marker Position with Drone Camera**
def Callback(msg):
    global x, y, z, theta

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    rot_q = msg.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta = abs(roll*180/3.14159)
    st = "({0} {1} {2} {3})".format(round(x, 3), round(y, 3), round(z, 3), round(theta, 3))
    print(st)
    
sub = rospy.Subscriber("/visualization_marker", Marker, Callback)
#*******************************************

time.sleep(2) #There was a delay, couldn't take off in the "try" loop.

if __name__ == '__main__':
    try:
        take_off.publish(to_send)
        time.sleep(5)
        
        while not rospy.is_shutdown():
            #Defining the angular speed
            #previous_theta = theta
            error = abs(180.0 - theta)
            
            #ca = pid_controller(error, theta, previous_theta)
            #ca = 0.1
            if error > 0.5: 
                ca = 0.0
            else: 
                ca = error
            #PID for z axe;l
            a = pid_controller(z, z, previous_pos_z)
            previous_pos_z = abs(z)
            #PID for x axe is the coordinate itself, can do the same for z, but well..PID for one speed is also good..
            b = -x
            #Keeping orientation
            if (theta > 185.0):
                speed.angular.z = 0.1
            elif (theta < 175.0):
                speed.angular.z = -0.1
            else: speed.angular.z = 0.0
            #Keeping distance tawards the marker
            if (z >= 1.1):
                speed.linear.x = a
            elif (z <= 0.9):
                speed.linear.x = -a
            else: speed.linear.x = 0.0
            #Keeping horizontal relation distance
            if (x >= 0.02):
                speed.linear.y = b
            elif (x <= -0.02):
                speed.linear.y = b
            else: speed.linear.y = 0.0
            #Keeping altitude
            if (y >= 0.3):
                speed.linear.z = -y
            elif (y <= -0.3):
                speed.linear.z = abs(y)
            else: speed.linear.z = 0.0
            
            pub.publish(speed)
            r.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass
