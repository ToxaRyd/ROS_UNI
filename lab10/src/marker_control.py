#! /usr/bin/env python

import rospy, time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from pynput.keyboard import Key, Listener
from tf.transformations import euler_from_quaternion

global x, y, z, theta, sp

x = 0.0
y = 0.0
z = 0.0

#Speed control
sp = 0.3
#*************

speed = Twist()
to_send = Empty()

pub = rospy.Publisher("/ar_tag/cmd_vel", Twist, queue_size = 1)
take_off = rospy.Publisher("/drone/takeoff", Empty, queue_size = 1)
land = rospy.Publisher("/drone/land", Empty, queue_size = 1)

rospy.init_node("lab10_marker")
r = rospy.Rate(10)

#Keyboard reading for AR Tag control********
def on_press(key):
    global speed, sp
    #print(key)
    if key == Key.page_up:
        take_off.publish(to_send)
    elif key == Key.page_down:
        land.publish(to_send)
    
    elif key == Key.up:
        speed.linear.y = -sp
    elif key == Key.down:
        speed.linear.y = sp
    
    elif key == Key.shift:
        speed.angular.z = -sp
    elif key == Key.alt:
        speed.angular.z = sp
    

    elif key == Key.left:
        speed.linear.x = sp
    elif key == Key.right:
        speed.linear.x = -sp
    return False

def on_release(key):
    #print("STOPPED!")
    global speed
    if key == Key.up:
        speed.linear.y = 0.0
    elif key == Key.down:
        speed.linear.y = 0.0
    
    elif key == Key.left:
        speed.linear.x = 0.0
    elif key == Key.right:
        speed.linear.x = 0.0

    elif key == Key.shift:
        speed.angular.z = 0.0
    elif key == Key.alt:
        speed.angular.z = 0.0
 
    return False
#*******************************************

#Reading Marker Position with Drone Camera**
def Callback(msg):
    global x, y, z

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    rot_q = msg.pose.orientation 
    roll, pitch, thetat = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta = (roll*180/3.14159)
    
    st = "({0} {1} {2} {3})".format(round(x, 3), round(y, 3), round(z, 3), round(theta, 3))
    #print(st)
    print(round(roll, 3), round(pitch, 3), round(thetat, 3))
    
sub = rospy.Subscriber("/visualization_marker", Marker, Callback)
#*******************************************

time.sleep(2) #There was a delay, couldn't take off in the "try" loop.

if __name__ == '__main__':
    try:
        take_off.publish(to_send)
        #land.publish(to_send)
        print('Marker control:\n   page_up --- take off\n   page_down --- land\n   arrows --- corresponding movement\n   ctrl and alt --- rotate\n')
        print('NOTE: keycontroll has a delay, short press adds speed but does not remove it\n1 second key press adds speed, on its release - removes it\n')
        while not rospy.is_shutdown():
            
            with Listener(
                on_press=on_press,
                on_release=on_release) as listener:
                listener.join()
                
            pub.publish(speed)
            r.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass
