#!/usr/bin/python
import rospy
from geometry_msgs.msg import Pose
import math
from tf.transformations import quaternion_from_euler

def publisher():
    pub = rospy.Publisher('turtlebot/goal_pose', Pose, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(2)

    defined_angle = 270

    target_rad = defined_angle*math.pi/180
    mouse = quaternion_from_euler(target_rad, 0, 0)

    print('Working, choosed angle in quaternion is {0}'.format(mouse[3]))
    
    while not rospy.is_shutdown():
        goal = Pose()
        goal.position.x = 3.0
        goal.position.y = 3.0
        goal.position.z = 0.0

        goal.orientation.x = 0.0
        goal.orientation.y = 0.0
        goal.orientation.z = 0.0
        goal.orientation.w = mouse[3]

        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass
