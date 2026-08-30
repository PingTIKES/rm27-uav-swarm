#!/usr/bin/env bash
# 编译（增量）并启动 4 机集群任务
set -e
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS_DIR"

source /opt/ros/humble/setup.bash
if [ ! -d install ]; then
    echo "[swarm] 首次编译工作空间..."
    colcon build --packages-up-to uav_bringup uav_swarm
fi
source install/setup.bash

echo "[swarm] 启动 4 机集群（起飞 -> 分区搜索 -> 汇聚 -> 返航降落）"
ros2 launch uav_bringup sim_swarm.launch.py num_uavs:=4
