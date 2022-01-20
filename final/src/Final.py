#! /usr/bin/env python

import rospy, time, math
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

#DEFINING VARIABLES
global x, y, z, theta, mode

x = 0.0
y = 0.0
z = 0.0

theta = 0.0

mode = False

global p, i, d

p = 0.0
i = 0.0
d = 0.0

global list_a, list_b, list_c

list_a = [0, 0, 0, 0, 0]
list_b = [0, 0, 0, 0, 0]
list_c = [0, 0, 0, 0, 0]

global  Kp_a, Ki_a, Kd_a, Kp_b, Ki_b, Kd_b, Kp_c, Ki_c, Kd_c

Kp_a = 0.0165
Ki_a = 0.002
Kd_a = 0.088

Kp_b = 0.02
Ki_b = 0.0
Kd_b = 0.0

Kp_c = 1.0
Ki_c = 0.3
Kd_c = 0.3

global to_send

to_send = Empty()

#DEFINING FUNCTIONS
def Callback_marker(msg):#For getting parameters from camera
    global x, y, z, theta, mode

    mode = True

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    rot_q = msg.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    #st = "\nTo target: {0} \nHorizontal: {1} \nAltitude: {2} \nTheta: {3}\n".format(round(z, 3), round(x, 3), round(y, 3), round(theta, 3))
    #print(st)

def pid_controller(error, list_i, Kp, Ki, Kd):#For defining the speed
    p = error*Kp

    i = 0
    for y in range(1, len(list_i)-1):
        i += list_i[y]*0.1
    i = i*Ki

    d = ((error - list_i[4])/10)*Kd

    return p+i+d

def limit(value, limit):
    limit = abs(limit)
    c = 1

    if value < 0:
        c = -1

    value = abs(value)

    if value > limit:
        value = limit

    return value*c

#Defining Subscribers, Publishers and initializing the node
rospy.init_node("Final")
r = rospy.Rate(10)

pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
take_off = rospy.Publisher("bebop/takeoff", Empty, queue_size = 1)
land = rospy.Publisher("bebop/land", Empty, queue_size = 1)

sub = rospy.Subscriber("/visualization_marker", Marker, Callback_marker)

time.sleep(3)
r = rospy.Rate(10)

if __name__ == '__main__':
    try:
        print("Taking off...\n")
        #take_off.publish(to_send)
        
        while not rospy.is_shutdown():
            speed = Twist()
            if mode == True:
                #Forward_speed
                error_a = z - 2
                
                a = pid_controller(error_a, list_a, Kp_a, Ki_a, Kd_a)

                list_a.pop(0)
                list_a.append(error_a)

                #Horizontal_speed
                error_b = -x

                b = pid_controller(error_b, list_b, Kp_b, Ki_b, Kd_b)

                list_b.pop(0)
                list_b.append(error_b)

                #Angular_speed
                if theta < 0:
                    error_c = -math.pi - theta
                else: error_c = math.pi - theta

                c = pid_controller(error_c, list_c, Kp_c, Ki_c, Kd_c)

                list_c.pop(0)
                list_c.append(error_c)

                #Publishing speeds
                speed.linear.x = 0.0#limit(a, 0.1)    
                speed.linear.y = 0.0#limit(b, 0.1)
                speed.angular.z = 0.0#limit(c, 0.1)


                if z <= 0.5:
                    speed.linear.x = 0.0     
                    speed.linear.y = 0.0
                    speed.angular.z = 0.0

                    print('*'*240)

                    land.publish(to_send)
       
                #limit(a, 0.1)
                print("\nForward speed: {0}; \nHorizontal speed: {1} \nAngular speed: {2}".format(a, b, c))
                #print("\nForward speed: {0}; \nHorizontal speed: {1} \nAngular speed: {2}".format(speed.linear.x, speed.linear.y, speed.angular.z))

                pub.publish(speed)
            else:
                speed.linear.x = 0.0       
                speed.linear.y = 0.0
                speed.angular.z = 0.0

                pub.publish(speed)
            r.sleep()
        rospy.spin()

    except KeyboardInterrupt:
        pass
