#! /usr/bin/env python

import rospy, time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from pynput.keyboard import Key, Listener

global x, y, z

x = 0.0
y = 0.0
z = 0.0

speed = Twist()
to_send = Empty()

pub = rospy.Publisher("/ar_tag/cmd_vel", Twist, queue_size = 1)
take_off = rospy.Publisher("/drone/takeoff", Empty, queue_size = 1)
land = rospy.Publisher("/drone/land", Empty, queue_size = 1)

rospy.init_node("lab10")
r = rospy.Rate(10)

#Keyboard reading for AR Tag control********
def on_press(key):
    global speed
    if key == Key.page_up:
        take_off.publish(to_send)
    elif key == Key.page_down:
        land.publish(to_send)
    
    elif key == Key.up:
        speed.linear.y = -1.0
    elif key == Key.down:
        speed.linear.y = 1.0
    
    elif key == Key.left:
        speed.linear.x = 1.0
    elif key == Key.right:
        speed.linear.x = -1.0
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
    return False
#*******************************************

#Reading Marker Position with Drone Camera**
def Callback(msg):
    global x, y, z

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    st = "\n ({0} {1} {2})".format(round(x, 3), round(y, 3), round(z, 3))
    print(st)
    
sub = rospy.Subscriber("/visualization_marker", Marker, Callback)
#*******************************************

time.sleep(2) #There was a delay, couldn't take off in the "try" loop.

if __name__ == '__main__':
    try:
        take_off.publish(to_send)
        #land.publish(to_send)
        print('Marker control:\n   page_up --- take off\n   page_down --- land\n   arrows --- corresponding movement\n')
        print('NOTE: keycontroll has delay, short press adds speed, 1 second key press adds speed, when released - removes it')
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
