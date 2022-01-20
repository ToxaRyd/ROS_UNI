# ROS_UNI
**Labs**

-Lab1 and 2

1. Teleoperating Turtlesim.
trajectory.
3. Getting Turtlebot 3 Gazebo Simulation running.
4. Move Turtlebot 3 in a circular trajectory.

-Lab3

1. Launch Turtlebot3 gazebo world
2. Perform the following obstacle avoidance task:
Go forward until you face an obstacle at 10 cm distance.
Make a 90 degree turn and try to past the obstacle.

-Lab4

1. Launch the turtlebot3 empty world.
2. Get a goal position in terms of x and y from the user through the terminal.
3. Move the robot to the goal.

-Lab5

1. A goal pose would be sent on a topic named "turtlebot/goal_pose" and you should receive the goal pose by subscribing to the topic.
2. Turtlebot 3 would start moving towards the goal using proportional controllers for both angular and linear velocities.

-Lab6

1. Replicate Lab 5 with a PID controller.
2. Plot the response and the set point using rqt_plot. Do you remember what were the response and set point?

-Lab7

1. Launch the simulation and see if the robot is already spawned.
2. Write a node to teleoperate the drone. The node should have capabilities of taking off, landing, and moving the drone.

-Lab8

Replicate lab 6 on the drone.

-Lab9

Using the ar_trac_alvar package and the camera of your laptop / usb_cam, detect ar markers.
1. Calibrate the camera
2. Show the ar_marker in rviz.
3. Create a node that prints the pose of the marker.

-Lab10

A controller for the drone to track the AR tag in the world.
