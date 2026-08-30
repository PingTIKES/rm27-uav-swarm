# RoboMaster 2027 四机集群仿真工作空间（uav_ws）

基于 `无人机27赛季框架.docx` 搭建的 **ROS2 Humble + PX4 SITL + Gazebo** 四机集群代码工作空间，
目标：在个人电脑上直接完成 **4 架小型无人机集群的仿真调试**，后续可平滑迁移到 RK3566 实机。

---

## 1. 工作空间结构

```
uav_ws/
├── README.md                  # 本文件（仿真调试手册）
├── setup_env.sh               # 一键安装依赖 + 克隆开源项目
├── scripts/
│   ├── start_sim_4uav.sh      # 启动 4 机 PX4 SITL + Gazebo + MicroXRCEAgent
│   ├── stop_sim.sh            # 结束仿真
│   └── run_swarm.sh           # 编译并启动集群任务
├── src/
│   ├── uav_msgs/              # 自定义消息/服务（Detection / DetectionArray / SetMission）
│   ├── uav_control/           # PX4 Offboard 桥（解锁、起飞、航点跟踪、降落）
│   ├── uav_swarm/             # 集群调度（区域划分、编队搜索、目标汇聚、返航）
│   ├── uav_perception/        # 仿真目标检测器 + YOLO/RKNN 实机节点（桩）
│   ├── uav_planning/          # 机间防碰安全监视 + VFH 避障模板（实机用）
│   ├── uav_mission/           # BehaviorTree.CPP 行为树（实机任务状态机）
│   ├── uav_localization/      # OpenVINS 配置与启动（实机视觉定位）
│   └── uav_bringup/           # launch 与参数（sim_swarm.launch.py 等）
└── third_party/               # setup_env.sh 克隆：px4_msgs / px4_ros_com / open_vins / BehaviorTree.CPP
```

## 2. 与框架文档的对应关系

| 框架模块 | 本工作空间实现 | 仿真替代方案 |
|---|---|---|
| px4_ros_com 桥接 | `uav_control/offboard_control.py`（px4_msgs + uXRCE-DDS） | 直接连 PX4 SITL |
| OpenVINS 双目 VIO | `uav_localization/`（配置就绪，实机启用） | 仿真用 EKF2 + GPS，话题接口不变 |
| rknn_yolo 目标检测 | `uav_perception/yolo_detector.py`（桩） | `sim_target_detector.py` 按真值+噪声发检测 |
| mission_control 行为树 | `uav_mission/behavior_trees/main_mission.xml` + `uav_swarm` 集群 FSM | 集群 FSM 全量仿真 |
| uav_planning 避障 | `uav_planning/`（高度分层 + 防碰监视，VFH 模板） | 仿真用高度分层即可 |
| 多机 namespace | 每机 `/uavN`（上层）↔ `/px4_N`（飞控） | 与实机完全一致 |

## 3. 环境要求

- Ubuntu 22.04（x86_64，仿真调试机；实机 RK3566 为 ARM64 同一套代码）
- ROS2 Humble
- 磁盘 ≥ 20 GB，内存 ≥ 8 GB（4 机 Gazebo 仿真）

## 4. 快速开始（仿真调试）

```bash
# 第 0 步：安装依赖并克隆开源项目（约 30-60 分钟，仅第一次）
cd uav_ws
chmod +x setup_env.sh scripts/*.sh   # 从 zip/git 检出后先补执行权限
./setup_env.sh

# 第 1 步：编译工作空间
source /opt/ros/humble/setup.bash
colcon build --packages-up-to uav_bringup uav_swarm

# 第 2 步：终端 A —— 启动 4 机仿真（PX4 SITL + Gazebo + DDS Agent）
./scripts/start_sim_4uav.sh

# 第 3 步：终端 B —— 启动集群任务（4 机自动起飞、分区搜索、发现目标后汇聚、返航降落）
./scripts/run_swarm.sh

# 结束后
./scripts/stop_sim.sh
```

预期现象：Gazebo 中 4 架 x500 依次解锁起飞至分层高度（2.0/2.5/3.0/3.5 m），
在各自分区做割草机式搜索；任一机发现目标后，四机汇聚到目标上空盘旋，随后返航降落。

## 5. 常用调试命令

```bash
# 查看某机飞控话题（飞控→ROS2）
ros2 topic echo /px4_1/fmu/out/vehicle_local_position
ros2 topic echo /px4_1/fmu/out/vehicle_status

# 查看集群状态与检测结果
ros2 topic echo /swarm/state
ros2 topic echo /uav1/detections

# 手动给 2 号机发一个航点（NED 本机坐标，z 负为向上）
ros2 topic pub /uav2/waypoint geometry_msgs/msg/Point "{x: 10.0, y: 2.0, z: -2.5}" -1

# 命令 3 号机立即降落
ros2 topic pub /uav3/command std_msgs/msg/String "{data: 'land'}" -1
```

## 6. 关键约定（务必先读）

1. **坐标系**：机载/飞控全部使用 PX4 本地 NED 系（北 x、东 y、下 z，**高度 = -z**）。
   集群公共坐标 = 各机本地坐标 + 出生点偏移（Gazebo `PX4_GZ_MODEL_POSE`，y 方向间隔 2 m）。
2. **命名空间**：上层节点 `/uav1..4`；飞控桥 `/px4_1..4`（PX4 SITL 实例 i 自动用 `px4_i`，
   其 MAVLink sysid = i+1，`uav_control` 已自动换算）。
3. **QoS**：所有 px4_msgs 话题必须 BEST_EFFORT + TRANSIENT_LOCAL（代码内已封装）。
4. **Offboard 心跳**：`OffboardControlMode` + `TrajectorySetpoint` 必须 ≥ 2 Hz 持续发布，
   否则飞控 0.5 s 后自动退出 Offboard（`offboard_control.py` 定时器已保证 10 Hz）。
5. **安全接管**：仿真中可随时 `Ctrl+C` 掉 `run_swarm.sh`，飞控会触发失控保护降落；
   实机务必保持 ET08 遥控器接管通道有效。

## 7. 迁移实机（RK3566）清单

- [ ] `setup_env.sh` 在 RK3566 上只装 ROS2 + CycloneDDS（**不要**装 Gazebo/PX4 SITL）
- [ ] 飞控串口 ↔ uXRCE-DDS Agent：`MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600`
- [ ] 启动 OpenVINS：`ros2 launch uav_localization openvins.launch.py`，
      输出 `/uavN/odom` 转 `vehicle_visual_odometry` 喂给 EKF2
- [ ] 用 `yolo_detector.py` 替换 `sim_target_detector.py`（加载 `models/yolov5s.rknn`）
- [ ] 集群通信改用 WiFi + CycloneDDS，关闭 `sim` 参数后同一套 launch 直接可用
