#!/bin/bash

# Source ROS workspace setup script
source /home/rm/catkin_livox_ros_driver2/devel/setup.sh

# Launch omni_gazebo gazebo
roslaunch simple_meca_car gazebo.launch &

# Wait for some time to ensure the previous launch finishes before starting the next one
sleep 5

# Run lidar2world node
rosrun lidar2world lidar2world_node &

# Run mpc_tracking node
rosrun mpc_tracking mpc_tracking_node &

roslaunch plan_manage topo_replan.launch &

# Launch omni_gazebo Pointcloud2scan.launch
roslaunch omni_gazebo Pointcloud2map.launch &

# 定义保存地图的函数
function save_map {
    echo "Saving map..."
    rosrun map_server map_saver map:=/projected_map -f /home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/mpc_track_fastplanner/src/omni_robot/Map/sim_map
    rosrun octomap_server octomap_saver -f /home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/mpc_track_fastplanner/src/omni_robot/Map/octomap.bt
    echo "Map saved."
}

# 循环直到收到SIGINT信号
while true; do
    # 每5秒保存一次地图
    save_map
    sleep 5
done

# Wait for all background jobs to finish
wait
