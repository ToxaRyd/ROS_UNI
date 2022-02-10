#! /usr/bin/env python

import rospy, time, math
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#DEFINING VARIABLES
global x, y, z, theta, mode, flight_mode, height
global param, count, c

param = [0, 0, 0, 0, 0]

x = 0.0
y = 0.0
z = 0.0
theta = 0.0

mode = False
flight_mode = False

height = 0.0

count = 0
c = 0.0

global p, i, d
p = 0.0
i = 0.0
d = 0.0

global list_a, list_b, list_c
list_a = [0, 0, 0, 0, 0]
list_b = [0, 0, 0, 0, 0]
list_c = [0, 0, 0, 0, 0]

global  Kp_a, Ki_a, Kd_a, Kp_b, Ki_b, Kd_b, Kp_c, Ki_c, Kd_c
Kp_a = 0.035 #WORKS 0.035
Ki_a = 0.25 #WORKS 0.25
Kd_a = 5.3 #WORKS 2.3

Kp_b = 0.05 #Before 0.1 very good for P config only 0.17
Ki_b = 0.2  #Before 0.1
Kd_b = 4.0  #Before 2.4

Kp_c = 0.7 #Norm alone 1.0
Ki_c = 0.3 #Norm alone 0.3
Kd_c = 0.3 #Norm alone 0.3

global to_send

to_send = Empty()

#DEFINING FUNCTIONS
def Callback_marker(msg):#For getting parameters from camera
    global x, y, z, theta, mode, param, timing

    param = [0, 0, 0, 0, 0]

    mode = True #Just to move only if the drone sees selected marker

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    rot_q = msg.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    param = [msg.id, x, y, z, theta]

    st = "\nTo target: {0} \nHorizontal: {1} \nAltitude: {2} \nTheta: {3}\n".format(round(z, 3), round(x, 3), round(y, 3), round(theta, 3))
    #print(st)

def Callback_bebop_odom(msg):
    global height

    height = msg.pose.pose.position.z
    #print(height)

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
rospy.init_node("RoboBiba")
r = rospy.Rate(10)

pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
pub_camera = rospy.Publisher("bebop/camera_control", Twist, queue_size = 1)

take_off = rospy.Publisher("bebop/takeoff", Empty, queue_size = 1)
land = rospy.Publisher("bebop/land", Empty, queue_size = 1)

sub = rospy.Subscriber("/visualization_marker", Marker, Callback_marker)
sub_height = rospy.Subscriber("/bebop/odom", Odometry, Callback_bebop_odom)

time.sleep(3)
r = rospy.Rate(10)

if __name__ == '__main__':
    try:
        speed_cam = Twist()
        speed_cam.angular.y = -40.0
        pub_camera.publish(speed_cam)

        print("Taking off...\n")

        take_off.publish(to_send)
        time.sleep(5)

        speed = Twist()
        while height < 1.7: #1.2
            speed.linear.z = 0.1
            pub.publish(speed)
        else: 
            speed.linear.z = 0.0
            pub.publish(speed)

        while not rospy.is_shutdown():

            #Counter of stable state
            if speed.linear.x <= 0.017 and speed.linear.y <= 0.017 and mode == True:
                count += 0.1
            else:
                count = 0.0
            
            #Changing flight mode
            if count > 10.0 and flight_mode == False:
                flight_mode = True
            
            if mode == True and param[0] == 4 and flight_mode == False:
                #Forward_speed
                error_a = param[3] - 1.3 #1.3
                
                a = limit(pid_controller(error_a, list_a, Kp_a, Ki_a, Kd_a), 0.2)

                if abs(error_a) <= 0.05:
                    a = 0.0
                
                list_a.pop(0)
                list_a.append(error_a)

                #Horizontal_speed
                error_b = -param[1]

                b = limit(pid_controller(error_b, list_b, Kp_b, Ki_b, Kd_b), 0.2)

                if abs(error_b) <= 0.05:
                    b = 0.0
                
                list_b.pop(0)
                list_b.append(error_b)

                #Angular_speed
                if param[4] < 0:
                    error_c = -math.pi - param[4]
                else: error_c = math.pi - param[4]

                c = limit(pid_controller(error_c, list_c, Kp_c, Ki_c, Kd_c), 0.2)

                #if abs(error_c) < 0.0:
                    #c = 0.0
                
                list_c.pop(0)
                list_c.append(error_c)

                #Publishing speeds
                speed.linear.x = a*0.2 #round(a, 3)    
                speed.linear.y = b*0.4 #round(b, 3)
                speed.angular.z = 0.0 #round(c, 3)

                #print("\n    Forward speed: {0} \n    Horizontal speed: {1} \n    Angular speed: {2}".format(a, b, c))
                print("\n    Forward speed: {0} \n    Horizontal speed: {1} \n    Angular speed: {2}".format(speed.linear.x, speed.linear.y, speed.angular.z))
                #print("\nTo target: {0} \nHorizontal: {1} \nAltitude: {2} \nTheta: {3}\n".format(round(param[3], 3), round(param[1], 3), round(param[2], 3), round(theta, 3)))
                #print("\n Start: {0} \n Update: {1}".format(start, timing))
                
                pub.publish(speed)
            
            elif flight_mode == True:

                if c == 0.0:
                    speed.linear.x = 0.0
                    speed.linear.y = 0.0
                    speed.angular.z = 0.0
                    pub.publish(speed)

                if c < 2.4: #3.1
                    speed.linear.x = 0.1
                    pub.publish(speed)
                    print("\n   {0}\n".format(c))
                else:
                    speed.linear.x = 0.0
                    pub.publish(speed)
                    land.publish(to_send)
                c += 0.1
            
            else:
                speed.linear.x = 0.0       
                speed.linear.y = 0.0
                speed.angular.z = 0.3
                mode = False
                pub.publish(speed)
                print("\n Marker Lost!")
            
            r.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass
