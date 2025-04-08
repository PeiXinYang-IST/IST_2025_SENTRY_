#!/bin/bash

# 启动第一个终端运行 Docker 容器中的 noetic.py
gnome-terminal --title="NOETIC" -- \
  bash -c "docker exec -it noetic /bin/bash -c 'cd IST_sentry_navigation/src/IST_2025_sentry && source ../../devel/setup.bash && python3 noetic.py; exec /bin/bash'"

sleep 2

# 启动第二个终端运行宿主机上的 humble.py
gnome-terminal --title="HUMBLE" -- \
  bash -c "cd /home/rm-nuc13/my-1 && source install/setup.bash && python3 humble.py; exec /bin/bash"

# 启动第三个终端运行宿主机上的 noetic.py
gnome-terminal --title="NOETIC" -- \
  bash -c "cd /home/rm-nuc13/my-1 && source install/setup.bash && python3 noetic.py; exec /bin/bash"

sleep 1

# 启动第四个终端运行Docker 容器上的 humble.py
gnome-terminal --title="HUMBLE" -- \
  bash -c "docker exec -it noetic /bin/bash -c 'cd IST_sentry_navigation/src/IST_2025_sentry && source ../../devel/setup.bash && python3 humble.py; exec /bin/bash'"

