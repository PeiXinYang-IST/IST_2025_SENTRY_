#!/bin/bash

# 主启动脚本 - start_all.sh

# 检测终端类型并启动第一个窗口
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal --title="导航系统" -- bash -c "./IST_sentry_navigation/src/IST_2025_sentry/run_navigation.sh; exec bash"
elif command -v xterm &> /dev/null; then
    xterm -title "导航系统" -e "./IST_sentry_navigation/src/IST_2025_sentry/run_navigation.sh; exec bash"
else
    echo "错误：未找到支持的终端模拟器 (gnome-terminal/xterm)"
    exit 1
fi

# 等待1秒确保第一个窗口启动
sleep 1

# 启动第二个窗口执行Python
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal --title="Python控制端" -- bash -c "./IST_sentry_navigation/src/IST_2025_sentry/run_python.sh; exec bash"
elif command -v xterm &> /dev/null; then
    xterm -title "Python控制端" -e "./IST_sentry_navigation/src/IST_2025_sentry/run_python.sh; exec bash"
else
    echo "错误：未找到支持的终端模拟器 (gnome-terminal/xterm)"
    exit 1
fi

echo "系统已启动，请查看两个终端窗口"