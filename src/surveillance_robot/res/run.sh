#!/bin/bash
# Source setup file
source ./devel/setup.bash
#source ./devel/setup_wired_connection.sh
echo $ROS_PACKAGE_PATH
roscd && cd ..

# Compile the project before running it
catkin_make

# Run RViz
rviz&

# Run external nodes
roscd surveillance_robot/res/corridor
rosrun map_server map_server corridor.yaml&
roscd && cd ..
rosrun amcl amcl _odom_alpha1:=5 _update_min_a:=0.17 _odoalpha2:=0.05 _odom_alpha3:=15 _odom_alpha4:=0.06 _update_min_d:=0.05 _min_particles:=800&

# Run surveillance_robot nodes
rosrun surveillance_robot surveillance_robot_decision_node&
rosrun surveillance_robot surveillance_robot_check_node&
rosrun surveillance_robot surveillance_robot_translation_node&
rosrun surveillance_robot surveillance_robot_rotation_node&
rosrun surveillance_robot surveillance_robot_obstacle_detection_node&
rosrun surveillance_robot surveillance_robot_local_planner_node&
rosrun surveillance_robot surveillance_robot_global_planner_node&

jobs -p
