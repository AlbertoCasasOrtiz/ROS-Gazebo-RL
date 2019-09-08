ROS-Gazebo RL
=============

Implementation of Watkins Q(λ) algorithm in ROS-Gazebo. The algorithms coordinates with another ROS node that acts as a pilot of the robot.

The robot used is [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) burger model and operates in maze world. The goal of the robot is to learn the minimal path from its origin to some point in the maze.

To execute the code clone this repository and follow the commands above:

* First move to catkin workspace and source the file devel/setup.bash.

<code>cd ~/catkin_ws && source devel/setup.bash</code>

* Make the project:

<code>catkin_make</code>

* Export the robot model:

<code>export TURTLEBOT3_MODEL=burger</code>

* Execute the world using roslaunch with any of the above commands:

<code>roslaunch reinforcenment_learning maze</code>

<code>roslaunch reinforcenment_learning maze_2</code>

* In other terminal, execute the reinforcenment_learning node:

<code>source devel/setup.bash</code>

<code>rosrun reinforcenment_learning reinforcenment_learning</code>

* In a third terminal, execute the robot_pilot node:

<code>source devel/setup.bash</code>

</code>rosrun robot_pilot robot_pilot</code>