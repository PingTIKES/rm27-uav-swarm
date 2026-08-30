#!/usr/bin/env bash
# =============================================================
# RoboMaster 2027 四机集群 —— 仿真环境一键安装脚本
# 适用：Ubuntu 22.04 x86_64 调试机（实机 RK3566 请见脚本尾部说明）
# 参考：PX4 Devguide / px4_ros_com / open_vins 官方文档
# =============================================================
set -e
WS_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "===== [1/6] 安装 ROS2 Humble 基础依赖 ====="
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions \
    ros-humble-ament-cmake-python ros-humble-geometry-msgs \
    ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-cv-bridge \
    git wget curl build-essential cmake

echo "===== [2/6] 克隆并安装 PX4-Autopilot（SITL + Gazebo 工具链） ====="
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    git clone --depth 1 --branch v1.15.4 https://github.com/PX4/PX4-Autopilot.git "$HOME/PX4-Autopilot" --recursive
fi
cd "$HOME/PX4-Autopilot"
# PX4 官方环境脚本：安装编译链 + Gazebo（22.04 对应 gz-garden）
bash ./Tools/setup/ubuntu.sh --no-nuttx
# 编译 SITL（首次约 20-40 分钟）
DONT_RUN=1 make px4_sitl_default

echo "===== [3/6] 安装 Micro-XRCE-DDS-Agent（PX4 <-> ROS2 桥） ====="
if [ ! -d "$HOME/Micro-XRCE-DDS-Agent" ]; then
    git clone --depth 1 --branch v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$HOME/Micro-XRCE-DDS-Agent"
    cd "$HOME/Micro-XRCE-DDS-Agent"
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig /usr/local/lib/
fi

echo "===== [4/6] 克隆 ROS2 依赖包到 third_party ====="
mkdir -p "$WS_DIR/third_party"
cd "$WS_DIR/third_party"
[ -d px4_msgs ]        || git clone --depth 1 --branch release/1.15 https://github.com/PX4/px4_msgs.git
[ -d px4_ros_com ]     || git clone --depth 1 https://github.com/PX4/px4_ros_com.git
[ -d BehaviorTree.CPP ]|| git clone --depth 1 --branch 4.6.2 https://github.com/BehaviorTree/BehaviorTree.CPP.git
# OpenVINS（实机视觉定位；仿真机可选，取消注释即可）
# [ -d open_vins ]     || git clone --depth 1 https://github.com/rpng/open_vins.git

echo "===== [5/6] 把 third_party 软链进 src（colcon 统一编译） ====="
cd "$WS_DIR/src"
for p in px4_msgs px4_ros_com BehaviorTree.CPP; do
    [ -e "$p" ] || ln -s "../third_party/$p" "$p"
done

echo "===== [6/6] 配置 CycloneDDS（嵌入式/多机更轻量） ====="
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
grep -q RMW_IMPLEMENTATION "$HOME/.bashrc" || \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> "$HOME/.bashrc"

echo ""
echo "================ 安装完成 ================"
echo "下一步："
echo "  source /opt/ros/humble/setup.bash"
echo "  cd $WS_DIR && colcon build --packages-up-to uav_bringup uav_swarm"
echo "  ./scripts/start_sim_4uav.sh      # 终端 A：4 机仿真"
echo "  ./scripts/run_swarm.sh           # 终端 B：集群任务"
echo ""
echo "【实机 RK3566】只需执行本脚本的 [1] [4] [6]，不装 PX4-SITL/Gazebo；"
echo "  Agent 改用串口：MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600"
