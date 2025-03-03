#!/bin/bash

# 启动Docker容器
sudo docker start noetic

# 允许Docker访问X服务器（GUI应用需要）
xhost +local:docker

# 在第一个终端中执行导航脚本
gnome-terminal --title="Navigation" -- bash -c \
    "sudo docker exec -it noetic /bin/bash -c 'cd IST_sentry_navigation && source devel/setup.bash && ./src/IST_sentry_navigation/navigation.sh'; exec bash"

# 在第二个终端中执行Python脚本
gnome-terminal --title="Noetic Python" -- bash -c \
    "sudo docker exec -it noetic /bin/bash -c 'cd IST_sentry_navigation && source devel/setup.bash && cd src/IST_sentry_navigation && python3 noetic.py'; exec bash"