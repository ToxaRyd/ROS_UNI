# ROS_UNI
Labs

-Lab1

roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
q
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'

--launching python bot program:

source ~/catkin_ws/devel/setup.bash

---Don't forget to make your node executable: 

chmod u+x ~/catkin_ws/src/lab1/src/spin.py

roscore
rosrun turtlesim turtlesim_node
rosrun lab1 spin.py

-Lab1_2

source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

source ~/catkin_ws/devel/setup.bash
rosrun lab1_2 spin_gazebo.py

