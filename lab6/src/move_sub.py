#!/usr/bin/python
import rospy, time
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist
from math import atan2, sqrt

global x, y
global theta
global goal
global speed_l, speed_a, K
global p, i , d, SP, PV, Kp, Ki, Kd, previous_pos
global pa, ia , da, SPa, PVa, Kpa, Kia, Kda, previous_pos_a

x = 0
y = 0
theta = 0

speed = Twist()
goal = Pose()

#******************************
#Linear
previous_pos = 0.0
p = 0.0
i = 0.0
d = 0.0

K = 0.15 #Coefficient to make the whole program working
Kp = 0.7
Ki = 0.5
Kd = 0.5

SP = 0.2 #Basically, tolerance with which we want our robot to be on the final point
#Angular
previous_pos_a = 0.0
pa = 0.0
ia = 0.0
da = 0.0

Kpa = 0.5
Kia = 0.5
Kda = 0.5

SPa = 0.3
#******************************

rospy.init_node("lab6", anonymous=True)
r = rospy.Rate(10)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

#Callbacks
def Callback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def Goal_Callback(msg):
    global goal
    goal = msg

#Subscriptions
sub = rospy.Subscriber("/odom", Odometry, Callback)
goal_sub = rospy.Subscriber("turtlebot/goal_pose", Pose, Goal_Callback)

c = True # Just for controlling info output

if __name__ == '__main__':
    try:
        time.sleep(5) # Just dealing with Subscription delay
        while not rospy.is_shutdown():
            angle_to_goal = atan2(goal.position.y - y, goal.position.x - x)

            #PID Linear***********
            PV = sqrt((goal.position.x - x)**2 + (goal.position.y - y)**2)
            error = PV - SP

            p = error*Kp
            #i += PV*Ki
            d = ((PV - previous_pos)/10)*Kd
            previous_pos = PV

            speed_l = K + p+i+d
            #*********************

            #PID Angular**********
            PVa = abs(theta - angle_to_goal)
            error_a = PVa - SPa

            pa = error_a*Kpa
            #ia += PV*Ki
            da = ((PVa - previous_pos_a)/10)*Kda
            previous_pos_a = PVa

            speed_a = K + pa+ia+da
            #*********************

            if c == True: #Just printing info
                #print(x, y, theta, goal.orientation.w)
                print(speed_l, speed_a)

            elif c == False:
                break

            if (abs(goal.orientation.w - theta) < 0.01 and PV <= 0.1):
                print("Position reached")
                speed_a = 0.0
                c = False

            if abs(angle_to_goal - theta) >= 0.2:
                speed.linear.x = 0.0
                speed.angular.z = speed_a
                if c == False:
                    speed.angular.z = 0.0

            else:
                speed.linear.x = speed_l
                speed.angular.z = 0.0   
            pub.publish(speed)
            r.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass
