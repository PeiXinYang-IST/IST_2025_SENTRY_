#!/bin/bash

# 设置脚本在遇到错误时终止执行
set -e

# 配置网络接口
sudo ifconfig enp86s0 192.168.1.50

# 切换到脚本所在的目录
cd "$(dirname "$0")"

# 源catkin工作空间
source /home/rm-nuc13/IST_sentry_navigation/devel/setup.sh

# 启动livox_ros_driver2节点
roslaunch livox_ros_driver2 msg_MID360.launch &
PID_LIVOX=$!
echo "Livox ROS Driver launched with PID: $PID_LIVOX"

# 等待1秒钟以确保第二个节点已经启动
echo "Waiting for 1 seconds to ensure Segmentation has started..."
sleep 1

# 启动fast_lio节点
echo "Starting Fast Lio..."
roslaunch fast_lio_localization sentry_build_map.launch &
PID_FAST_LIO=$!
echo "Fast Lio launched with PID: $PID_FAST_LIO"

# 定义保存地图的函数
function save_map {
    echo "Saving map..."
    rosrun map_server map_saver map:=/projected_map -f /home/rm-nuc13/IST_sentry_navigation/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/Map/demo2
    rosrun map_server map_saver map:=/projected_map -f /home/rm-nuc13/tutututu/demo
    rosrun octomap_server octomap_saver -f /home/rm-nuc13/IST_sentry_navigation/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/Map/octomap.bt
    echo "Map saved."
}

# 定义一个函数来处理SIGINT信号
function handle_sigint {
    echo "Caught SIGINT. Stopping all nodes..."
    kill -SIGINT $PID_LIVOX
    kill -SIGINT $PID_FAST_LIO
    echo "All nodes have been stopped."
    save_map # Save map one last time on exit
    exit 1
}

# 捕获SIGINT信号，并调用handle_sigint函数
trap 'handle_sigint' SIGINT

# 循环直到收到SIGINT信号
while true; do
    # 每5秒保存一次地图
    save_map
    sleep 5
done