#!/bin/bash

# 定义一个数组，包含所有需要重新安装的包
packages=(
    "python3-colcon-defaults"
    "libgraphicsmagick-q16-3"
    "python3-pydot"
    "ros-noetic-laser-filters"
    "ros-foxy-ament-cmake-export-interfaces"
    "libavresample-dev"
    "python3-colcon-library-path"
    "ros-foxy-tango-icons-vendor"
    "ros-foxy-uncrustify-vendor"
    "ros-noetic-qt-gui"
    "libode-dev"
    "ros-noetic-trajectory-msgs"
    "ros-foxy-logging-demo"
    "ros-noetic-navigation-experimental"
    "libdc1394-22"
    "ros-foxy-nav2-msgs-dbgsym"
    "libopencv-core-dev"
    "ros-foxy-tf2-ros"
    "ros-noetic-rospy"
    "ros-foxy-rclcpp-action"
    "liblept5"
    "libignition-math6"
    "python3-pyside2.qtwidgets"
    "libsocket++1"
    "ros-foxy-joy"
    "ros-foxy-examples-rclcpp-multithreaded-executor"
    "python3-colcon-pkg-config"
    "ros-foxy-nav2-behavior-tree"
    "python3-html5lib"
    "ros-noetic-nodelet"
    "python3-pydocstyle"
    "libopencv-photo4.2"
    "ros-noetic-rqt-common-plugins"
    "ros-noetic-genmsg"
    "ros-foxy-ament-lint"
)

# 更新包列表
sudo apt-get update

# 安装所有包
for package in "${packages[@]}"; do
    echo "Installing $package..."
    sudo apt-get install -y $package
done

echo "All packages have been installed."