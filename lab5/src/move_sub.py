#!/usr/bin/python
import rospy, time
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist
from math import atan2

global x
global y
global theta
global goal
global speed_r

x = 0
y = 0
theta = 0

speed = Twist()
goal = Pose()

rospy.init_node("lab5", anonymous=True)
r = rospy.Rate(10)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

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

sub = rospy.Subscriber("/odom", Odometry, Callback)
goal_sub = rospy.Subscriber("turtlebot/goal_pose", Pose, Goal_Callback)

speed_r = 0.2
c = True # Just for controlling info output

if __name__ == '__main__':
    try:
        time.sleep(5) # Just dealing with Subscription delay
        while not rospy.is_shutdown():
            angle_to_goal = atan2(goal.position.y - y, goal.position.x - x)

            if c == True: #Just printing info
                print(x, y, theta, goal.orientation.w)

            if (abs(goal.orientation.w - theta) < 0.01 and (abs(goal.position.x - x) < 0.2 and abs(goal.position.y - y) < 0.2)):
                print("Position reached")
                speed_r = 0.0
                c = False

            if abs(angle_to_goal - theta) >= 0.2:
                speed.linear.x = 0.0
                speed.angular.z = speed_r

            else:
                speed.linear.x = 0.3
                speed.angular.z = 0.0   
            pub.publish(speed)
            r.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass
