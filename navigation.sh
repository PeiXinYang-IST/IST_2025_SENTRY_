#!/bin/bash

# 设置脚本在遇到错误时终止执行
set -e

# 配置网络接口
sudo ifconfig enp88s0 192.168.1.50

# 切换到脚本所在的目录
cd "$(dirname "$0")"

# 源catkin工作空间
source /home/rm/catkin_livox_ros_driver2/devel/setup.sh

LIVOX_STARTED=0

# 启动节点的函数
restart_nodes() {
    echo "Restarting all nodes..."

    # 杀死旧的节点进程，除了Livox节点
    kill_processes

 # 如果Livox节点没有启动，则启动它
    if [ $LIVOX_STARTED -eq 0 ]; then
        roslaunch livox_ros_driver2 msg_MID360.launch &
        PID_LIVOX=$!
        echo "Livox ROS Driver started with PID: $PID_LIVOX"
        LIVOX_STARTED=1
    fi

    sleep 1

    # 启动fast_lio节点
    roslaunch fast_lio_localization sentry_localize.launch &
    PID_FAST_LIO=$!
    echo "Fast Lio relaunched with PID: $PID_FAST_LIO"

    sleep 1
#real
    # roslaunch plan_manage real.launch &
    # PID_FAST_PLANNER=$!
    # echo "Fast planner relaunched with PID: $PID_FAST_PLANNER"

    # 启动NAV节点
    # roslaunch sentry_nav sentry_movebase.launch &
    # PID_NAV=$!
    # echo "NAV relaunched with PID: $PID_NAV"

    # 启动Serial节点
    # roslaunch sentry_serial serial.launch &
    # PID_SERIAL=$!
    # echo "Serial relaunched with PID: $PID_SERIAL"
}

# 杀死进程的函数
kill_processes() {
    # 杀死除了Livox节点之外的所有节点进程
    if [ ! -z "$PID_FAST_LIO" ]; then
        kill -SIGINT $PID_FAST_LIO
    fi
    if [ ! -z "$PID_NAV" ]; then
        kill -SIGINT $PID_NAV
    fi
    if [ ! -z "$PID_SERIAL" ]; then
        kill -SIGINT $PID_SERIAL
    fi
    wait # 确保所有子进程都结束
}

# 首次启动所有节点
restart_nodes

# 监听特定话题并检测特定信息
echo "Monitoring topic /MY_ICP/restart for restart signal..."
while true; do
    # 使用rostopic监听并检测 "True" 字符串
    rostopic echo /MY_ICP/restart | grep -q "True"
    if [ $? -eq 0 ]; then
        restart_nodes
    fi
    sleep 1
done &

# Ctrl+C 按下时退出脚本
trap 'echo "Exiting script..."; kill_processes; exit;' SIGINT

# 无限循环，等待用户按Ctrl+C退出
echo "All nodes are running in the background. Press Ctrl+C to exit."
wait