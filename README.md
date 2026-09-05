# RoboMaster 2027 四机集群仿真工作空间（uav_ws）

基于 `无人机27赛季框架.docx` 搭建的 **ROS2 Humble + PX4 SITL + Gazebo** 四机集群代码工作空间。
目标：在个人电脑上（不要任何真飞机）直接完成 **4 架小型无人机集群的仿真调试**，
逻辑验证后平滑迁移到 RK3566 实机。

---

## 目录

- [1. 系统总览与数据流](#1-系统总览与数据流)
- [2. 功能包详解](#2-功能包详解)
- [3. 仿真环境开启流程（详细版）](#3-仿真环境开启流程详细版)
- [4. 运行中调试命令](#4-运行中调试命令)
- [5. 常见排错](#5-常见排错)
- [6. 关键约定](#6-关键约定)
- [7. 迁移实机（RK3566）清单](#7-迁移实机rk3566清单)

---

## 1. 系统总览与数据流

```
                 Gazebo 仿真世界（4 架虚拟 x500）
                    │  传感器/电机
        ┌───────────┼───────────┬───────────┐
     PX4 SITL 1  PX4 SITL 2  PX4 SITL 3  PX4 SITL 4     ← 4 个虚拟飞控进程
        │           │           │           │
        └───────────┴─────┬─────┴───────────┘
                   MicroXRCEAgent（udp4:8888，飞控↔ROS2 翻译官）
                          │
        /px4_1/fmu/*  /px4_2/fmu/*  /px4_3/fmu/*  /px4_4/fmu/*
                          │
        ┌─────────────────┼─────────────────┐
   uav_control×4（飞行员）              uav_perception×4（眼睛）
   /uavN/state 汇报                     /uavN/detections 检测
        │                                    │
        └──────────> swarm_coordinator（指挥官，1 个实例）<──────────┘
                          │ /uavN/waypoint 下发航点
                   collision_monitor（安全员，1 个实例）
```

- **仿真替代关系**：OpenVINS → PX4 EKF2+GPS（接口不变）；YOLO 检测 → 真值+噪声的仿真检测器（消息格式不变）
- **命名空间**：上层每机 `/uav1..4`，飞控侧 `/px4_1..4`，与实机完全一致

## 2. 功能包详解

### 2.1 uav_msgs —— 自定义通信协议（地基）

整个系统的"语言"，其他所有包都依赖它。

| 文件 | 作用 |
|---|---|
| `msg/Detection.msg` | 单个目标检测结果：类别名、置信度、像素检测框、解算出的目标 3D 位置（NED 系） |
| `msg/DetectionArray.msg` | 一帧图像的全部检测结果（数组） |
| `srv/SetMission.srv` | 地面站/调度下发任务的服务接口：0=待机 1=区域搜索 2=定点汇聚 3=返航 4=紧急降落 |

**为什么单独成包**：仿真检测器和实机 YOLO 检测器发布同一种消息，
集群调度因此完全不用区分"仿真还是实机"——这是先在仿真调通逻辑、再无缝上实机的关键。

### 2.2 uav_control —— PX4 Offboard 桥（每机的"飞行员"）

每架无人机运行一个实例（命名空间 `/uav1`~`/uav4`），对接飞控命名空间 `/px4_1`~`/px4_4`。
内部是一个五态状态机：

```
INIT(发心跳) → ARMING(切Offboard+解锁) → TAKEOFF(升到指定高度) → MISSION(跟踪航点) → LAND → IDLE
```

- **向上接口**（给集群调度用）：订阅 `~/waypoint`（航点）、`~/command`（land 指令），发布 `~/state`（状态汇报）
- **向下接口**（给飞控）：发布 `offboard_control_mode` + `trajectory_setpoint` + `vehicle_command`，
  订阅 `vehicle_local_position` + `vehicle_status`
- **已处理的坑**：
  - Offboard 心跳必须 ≥2 Hz，否则飞控 0.5 秒后自动退出 Offboard（本节点定时器保证 10 Hz）
  - 切换 Offboard 模式前需先发约 1 秒心跳，飞控才接受
  - 多机时 MAVLink sysid = SITL 实例号 + 1（节点内自动换算）
  - px4_msgs 话题必须 BEST_EFFORT + TRANSIENT_LOCAL QoS（已封装为 `px4_qos()`）

### 2.3 uav_swarm —— 集群调度（"指挥官"，只运行一个实例）

对应框架文档 4.5 节，整个系统的核心大脑，全局状态机：

```
WAIT_TAKEOFF → SEARCH → CONVERGE → RETURN → LAND → DONE
```

- **WAIT_TAKEOFF**：等全部 4 机的 offboard 节点汇报 MISSION（起飞完成）
- **SEARCH**：把搜索区域（默认 x∈[10,22]，y∈[-1,7]）按东向等分为 4 条带，
  每机在各自条带内飞割草机航线；四机高度分层（2.0/2.5/3.0/3.5 m）天然防碰
- **CONVERGE**：任一机 `/uavN/detections` 出现 `target` 且置信度 >0.5，
  四机立刻汇聚到目标四周（水平错开 1.5 m、保持各自高度层）盘旋 15 秒
- **RETURN / LAND**：各机返回出生点上空，到位后统一下发 land 指令
- **坐标换算**：每机本地坐标原点在自己出生点，调度器内部维护
  "公共坐标系 = 本机坐标 + 出生点偏移"，下发航点时自动减回偏移——调参时只需想公共系
- **超时保护**：单航点 30 秒未到达强制切下一个，避免某机卡死拖住全队

### 2.4 uav_perception —— 感知（仿真/实机双实现）

| 节点 | 用途 |
|---|---|
| `sim_target_detector.py` | **仿真专用**：目标真值（默认 (16,3,0)，在搜索区中部）+ 高斯噪声，本机进入 8 m 探测圈时以 5 Hz 发布 DetectionArray |
| `yolo_detector.py` | **实机桩代码**：接口与仿真检测器完全一致，注释中标注了 RK3566 上加载 `yolov5s.rknn`、RKNN 推理、NMS 后处理的替换位置（参考 `airockchip/rknn_model_zoo`） |

### 2.5 uav_planning —— 规划与安全

- `collision_monitor.py`：**集群级防碰兜底**。10 Hz 两两计算机间 3D 距离：
  <1.5 m 发告警到 `/swarm/collision_warning`；<0.8 m 直接沿连线反向把两机各拉开 1 m
  （2 秒冷却，避免与调度器抢航点）。仿真靠高度分层基本不触发，实机是安全底线
- `vfh_planner.py`：VFH+ 避障**模板**（实机接双目深度点云用），当前实现为
  "直通 + 斥力修正"，注释里写了升级完整 VFH+（直方图阈值化→候选谷→代价选向）的路径

### 2.6 uav_mission —— 行为树（实机任务层）

`behavior_trees/main_mission.xml`：BehaviorTree.CPP v4 语法，逻辑为
`ReactiveSequence` 包裹的「电量/视觉安全检查 → 起飞 →（发现目标则汇聚，否则搜索）→ 返航 → 降落」。
仿真中这套逻辑由 swarm_coordinator 的 FSM 等价实现；上实机时用 C++ 注册
各 Action/Condition 节点（`mission_control.cpp`）即可替换，CMakeLists 中已预留编译位置。

### 2.7 uav_localization —— OpenVINS 视觉定位（实机）

仿真用 PX4 EKF2+GPS 定位，**不需要本包**；实机无 GPS 环境下启用。
`openvins_params.yaml` 已按框架第 7 节预设：640×400 双目、特征点 ≤150、
输出 20 Hz、关闭回环，Kalibr 标定结果直接填入。输出 `/uavN/odom` 转
`vehicle_visual_odometry` 喂给飞控 EKF2 融合（飞控参数 `EKF2_EV_CTRL=15`）。

### 2.8 uav_bringup —— 启动与参数（总开关）

- `launch/sim_swarm.launch.py`：**仿真一键启动**，拉起 4 个 offboard 节点 +
  4 个仿真检测器 + 集群调度 + 防碰监视，`num_uavs`/`sim` 等参数可调
- `launch/uav_bringup.launch.py`：实机单机启动（`auto_takeoff` 默认 False，遥控器先验证）
- `config/params.yaml`：**调参唯一入口**——搜索区域、高度层、防碰距离、
  仿真目标位置都在这一个文件里

---

## 3. 仿真环境开启流程（详细版）

> 前提：一台 **x86 电脑、Ubuntu 22.04、ROS2 Humble、≥8GB 内存、≥20GB 磁盘**
> （不是在 RK3566 上跑）。

### 第 0 步：一次性环境安装（约 30~60 分钟，只做一次）

```bash
git clone https://github.com/PingTIKES/rm27-uav-swarm.git   # 或解压 zip
cd rm27-uav-swarm        # 即 uav_ws 工作空间
chmod +x setup_env.sh scripts/*.sh
./setup_env.sh
```

脚本自动完成 6 件事（详见脚本内注释）：
装 ROS2 依赖 → 克隆 PX4-Autopilot v1.15.4 并编译 SITL（虚拟飞控）→
编译安装 Micro-XRCE-DDS-Agent（飞控↔ROS2 桥）→
克隆 px4_msgs/px4_ros_com/BehaviorTree.CPP 到 third_party → 软链进 src → 配置 CycloneDDS。

### 第 1 步：编译工作空间（以后每次改代码都要做）

```bash
source /opt/ros/humble/setup.bash
cd ~/rm27-uav-swarm
colcon build --packages-up-to uav_bringup uav_swarm
```

### 第 2 步：终端 A —— 启动 4 机仿真

```bash
./scripts/start_sim_4uav.sh
```

脚本依次：启动 PX4 实例 1（连带 Gazebo 服务器）→ 间隔 2 秒启动实例 2/3/4
（standalone 接入，出生点东向各错 2 m）→ 启动 MicroXRCEAgent
（udp4:8888，自动接管全部 4 个实例）。Gazebo 窗口中应出现 4 架 x500。

**验证**（新开终端）：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep px4_
# 应看到 /px4_1/fmu/out/... ~ /px4_4/fmu/out/... 四组话题
```

### 第 3 步：终端 B —— 启动集群任务

```bash
./scripts/run_swarm.sh
```

### 第 4 步：观察预期行为

4 架 x500 依次解锁 → 爬升到 2.0/2.5/3.0/3.5 m 分层高度 →
各自飞向分到的条带做割草机搜索 → 飞到 (16,3) 附近的飞机"发现"目标 →
四机汇聚盘旋 15 秒 → 各自返回出生点上空 → 降落上锁 → 终端打印"任务结束"。

### 第 5 步：结束仿真

终端 A 按 `Ctrl+C`，或任意终端执行 `./scripts/stop_sim.sh`。

## 4. 运行中调试命令

```bash
ros2 topic echo /swarm/state                               # 集群当前阶段
ros2 topic echo /uav2/state                                # 2 号机状态机
ros2 topic echo /swarm/collision_warning                   # 防碰告警
ros2 topic echo /px4_1/fmu/out/vehicle_local_position      # 1 号机位置（NED）
ros2 topic hz /px4_1/fmu/in/offboard_control_mode          # Offboard 心跳（应≈10Hz）

# 手动给 2 号机发一个航点（NED 本机坐标，z 负为向上）
ros2 topic pub /uav2/waypoint geometry_msgs/msg/Point "{x: 10.0, y: 2.0, z: -2.5}" -1

# 命令 3 号机立即降落
ros2 topic pub /uav3/command std_msgs/msg/String "{data: 'land'}" -1
```

## 5. 常见排错

| 现象 | 原因与处理 |
|---|---|
| `ros2 topic list` 没有 px4 话题 | Agent 没连上：看终端 A 输出；`tail /tmp/px4_instance_1.log` 查 PX4 日志 |
| 飞机不起飞、卡在 ARMING | 心跳没通：确认 launch 是在 `start_sim_4uav.sh` **之后**启动；检查 offboard 心跳频率是否 ≈10 Hz |
| Gazebo 里只有 1 架飞机 | 实例启动太快抢模型名：把脚本里的 `sleep 2` 调大 |
| 改了 params.yaml 没生效 | launch 读的是 install 下的副本：重新 `colcon build` 并 `source install/setup.bash` |
| 某机不跟航点 | 确认航点发到了该机的命名空间 `/uavN/waypoint`，且坐标是该机**本地系**（公共系坐标需减出生点偏移） |

## 6. 关键约定

1. **坐标系**：全部使用 PX4 本地 NED 系（北 x、东 y、下 z，**高度 = -z**）。
   集群公共坐标 = 各机本地坐标 + 出生点偏移（y 方向间隔 2 m）。
2. **命名空间**：上层节点 `/uav1..4`；飞控桥 `/px4_1..4`（实例 i 的 sysid = i+1，已自动换算）。
3. **QoS**：所有 px4_msgs 话题必须 BEST_EFFORT + TRANSIENT_LOCAL（代码内已封装）。
4. **Offboard 心跳**：`OffboardControlMode` + `TrajectorySetpoint` 必须 ≥2 Hz 持续发布。
5. **安全接管**：仿真中 `Ctrl+C` 掉 run_swarm 后飞控会触发失控保护降落；
   实机务必保持 ET08 遥控器接管通道有效。

## 7. 迁移实机（RK3566）清单

- [ ] `setup_env.sh` 在 RK3566 上只执行第 1、4、6 步（**不要**装 Gazebo/PX4 SITL）
- [ ] 飞控串口 ↔ uXRCE-DDS Agent：`MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600`
- [ ] 启动 OpenVINS：`ros2 launch uav_localization openvins.launch.py`，
      输出 `/uavN/odom` 转 `vehicle_visual_odometry` 喂给 EKF2（`EKF2_EV_CTRL=15`）
- [ ] 用 `yolo_detector.py` 替换 `sim_target_detector.py`（加载 `models/yolov5s.rknn`）
- [ ] 集群通信改用 WiFi + CycloneDDS
- [ ] 实机首飞用 `uav_bringup.launch.py`（`auto_takeoff:=False`，遥控器接管验证后再放开）
