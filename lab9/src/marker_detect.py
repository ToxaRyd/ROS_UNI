#!/usr/bin/python
import rospy
from visualization_msgs.msg import Marker

global x, y, z

x = 0.0
y = 0.0
z = 0.0

rospy.init_node("lab9", anonymous=True)
r = rospy.Rate(10)

def Callback(msg):
    global x, y, z

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    print(x, y, z)
#Sorry for being sometimes in slow mode...
sub = rospy.Subscriber("/visualization_marker", Marker, Callback)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            r.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        pass
