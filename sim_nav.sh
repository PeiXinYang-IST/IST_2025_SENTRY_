#!/bin/bash

# Source ROS workspace setup script
source /home/rm/catkin_livox_ros_driver2/devel/setup.sh

# Launch omni_gazebo gazebo
roslaunch simple_meca_car gazebo.launch &
# roslaunch omni_gazebo gazebo.launch &
# Wait for some time to ensure the previous launch finishes before starting the next one
sleep 5

# Launch nav
roslaunch omni_gazebo nav.launch &

# Run lidar2world node
rosrun lidar2world lidar2world_node &

# Run mpc_tracking node
# rosrun mpc_tracking mpc_tracking_node &

rosrun fast_lio_localization sim_obstacle_map_cloud &

sleep 1

roslaunch plan_manage topo_replan.launch &
# roslaunch plan_manage real.launch &

rosrun fast_lio_localization sim_point_cloud_deal

# Wait for all background jobs to finish
wait
