#!/bin/bash
# 在新终端中执行Python脚本
gnome-terminal -- \
  bash -c "sudo docker exec -it noetic /bin/bash -c '\
    cd IST_sentry_navigation/src/IST_2025_sentry && \
    source ../../devel/setup.bash && \
    python3 noetic.py;\
    exec /bin/bash'; read -p 'Press enter to close...'"