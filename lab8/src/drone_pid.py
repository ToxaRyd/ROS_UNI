#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Empty

global x, y
global theta
global goal
global kx, ky
global p, i, d, Kp, Kd
global previous_pos_x, previous_pos_y

x = 0 
y = 0
theta = 0

p = 0.0
i = 0.0
d = 0.0

kx = 1.0
ky = 1.0
#Coefficients for controlling the tolerance (My description :D)
Kp = 0.4
Kd = 0.4

previous_pos_x = 0.0
previous_pos_y = 0.0

speed = Twist()
goal = Pose()

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
take_off = rospy.Publisher("/drone/takeoff", Empty, queue_size = 1)
land = rospy.Publisher("/drone/land", Empty, queue_size = 1)

to_send = Empty()

#PID Function
def pid_controller(error, first, second):

    p = error*Kp
    d = ((first - second)/10)*Kd
    PID = p + d

    return PID

rospy.init_node("lab8", anonymous=True)
r = rospy.Rate(10)

#Callbacks
def Callback(msg):
    global x
    global y
    global z
    global theta

    x = msg.position.x
    y = msg.position.y
    z = msg.position.z
    rot_q = msg.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

sub = rospy.Subscriber("/drone/gt_pose", Pose, Callback)

#Decided to make it this way
goal.position.x = float(input("Type x coordinate: "))
goal.position.y = float(input("Type y coordinate: "))

#This is made to allow usage of negative coordinates
if goal.position.x < 0:
    kx = -1.0
if goal.position.y < 0:
    ky = -1.0

rospy.sleep(2)

if __name__ == '__main__':
    try:
        print("Moving!")
        take_off.publish(to_send)

        while not rospy.is_shutdown():
            #PID for x axe
            error_x = abs(goal.position.x) - abs(x)
            a = kx*pid_controller(error_x, x, previous_pos_x)
            speed.linear.x = a
            previous_pos_x = x
            #PID for y axe
            error_y = abs(goal.position.y) - abs(y)
            b = ky*pid_controller(error_y, y, previous_pos_y)
            speed.linear.y = b
            previous_pos_y = y
            #When position reached
            if (error_x <= 0.01 and error_y <= 0.01):
                land.publish(to_send)
                print("Position reached!")
                break
            #Just printing speed and after, coordinates with angle
            print(round(a, 2), round(b, 2))
            print(x, y, theta)
            
            pub.publish(speed)
            r.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass