#!/bin/bash

# 启动Docker容器
docker start noetic

# 允许Docker访问本地X服务器
xhost +local:docker

# 在容器内执行导航系统启动命令
docker exec -it noetic /bin/bash -c "\
  cd IST_sentry_navigation && \
  source devel/setup.bash && \
  ./src/IST_2025_sentry/navigation.sh; \
  exec /bin/bash"  # 保持终端打开

