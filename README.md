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
                 Gazebo 仿真世界（RM2025 赛场，4 架虚拟 x500）
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
| --- | --- |
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
- **SEARCH**：把搜索区域（默认 x∈[-10,8]，y∈[-6,6]，覆盖赛场中场至红方半场）按东向等分为 4 条带，
每机在各自条带内飞割草机航线；四机高度分层（2.0/2.5/3.0/3.5 m）天然防碰
- **CONVERGE**：任一机 `/uavN/detections` 出现 `target` 且置信度 >0.5，
四机立刻汇聚到目标四周（水平错开 1.5 m、保持各自高度层）盘旋 15 秒
- **RETURN / LAND**：各机返回出生点上空，到位后统一下发 land 指令
- **静态避障（A\*）**：每个任务航点（搜索/汇聚/返航）都先经
`uav_planning.field_map` 在赛场占据栅格（默认 RMUC2025 真实场地离线栅格）
上规划、视线拉直后拆成子航点依次下发，自动绕开资源岛/高地/环公路高架等
场地障碍；落入障碍的航点自动吸附到最近自由点。启动日志会打印
"A\* 避障已启用，地图来源：…"；uav_planning 不可用时退化为直航并告警
- **坐标换算**：每机本地坐标原点在自己出生点，调度器内部维护
"公共坐标系 = 本机坐标 + 出生点偏移"，下发航点时自动减回偏移——调参时只需想公共系
- **超时保护**：单航点 30 秒未到达强制切下一个，避免某机卡死拖住全队

### 2.4 uav_perception —— 感知（仿真/实机双实现）

| 节点 | 用途 |
| --- | --- |
| `sim_target_detector.py` | **仿真专用**：目标真值（默认 NED (-9,2,0)，红方半场，对应世界中绿色立柱）+ 高斯噪声，本机进入 8 m 探测圈时以 5 Hz 发布 DetectionArray |
| `yolo_detector.py` | **实机桩代码**：接口与仿真检测器完全一致，注释中标注了 RK3566 上加载 `yolov5s.rknn`、RKNN 推理、NMS 后处理的替换位置（参考 `airockchip/rknn_model_zoo`） |

### 2.5 uav_planning —— 规划与安全

- `field_map.py`：**赛场占据栅格地图 + A\***（纯算法模块，不依赖 ROS）。
  默认加载离线栅格 `maps/rmuc_2025_occ.npz`——由 RMUC2025 真实场地网格
  （`worlds/models/rmuc_2025`，源自 SMBU-PolarBear rmu_gazebo_simulator）
  按 z∈[0.35, 3.8] m 光栅化并膨胀 0.5 m 生成；文件缺失时回退到与简化场地
  `worlds/rm2025_field.sdf` 对应的内置解析障碍。8 连通 A* + 视线拉直平滑。
  实机演进时接口不变，障碍来源换成 D430i 深度点云局部建图即可
- `goal_planner.py`：**RViz 打点导航**。订阅 RViz "2D Nav Goal" 的 `/goal_pose`，
  A* 规划后把路径拆成航点序列依次下发给指定无人机的 offboard 节点；
  同时发布 `/field_map` 占据栅格、`/planned_path` 路径、`/goal_marker` 目标标记
- `pose_tf_publisher.py`：**位姿→TF+标记桥**。把 4 机的 PX4 本地 NED 位置换算到
  公共系，广播 `map->uavN` TF 并发布机身/机头/机号标记供 RViz 显示
- `collision_monitor.py`：**集群级防碰兑底**。10 Hz 两两计算机间 3D 距离：
<1.5 m 发告警到 `/swarm/collision_warning`；<0.8 m 直接沿连线反向把两机各拉开 1 m
（2 秒冷却，避免与调度器抢航点）。仿真靠高度分层基本不触发，实机是安全底线
- `vfh_planner.py`：VFH+ 避障**模板**（实机接双目深度点云用），当前实现为
"直通 + 斥力修正"，注释里写了升级完整 VFH+（直方图阈值化→候选谷→代价选向）的路径

### 2.6 uav_mission —— 行为树（实机任务层）

`behavior_trees/main_mission.xml`：BehaviorTree.CPP v4 语法，逻辑为
`ReactiveSequence` 包裹的「电量/视觉安全检查 → 起飞 →（发现目标则汇聚，否则搜索）→ 返航 → 降落」。
仿真中这套逻辑由 swarm_coordinator 的 FSM 等价实现；上实机时用 C++ 注册
各 Action/Condition 节点（`mission_control.cpp`）即可替换，CMakeLists 中已预留编译位置。

### 2.7 uav_localization —— OpenVINS 视觉定位（实机 + 仿真测试）

实机无 GPS 环境下启用（仿真默认仍用 PX4 EKF2+GPS，OpenVINS 仅作测试）。
`openvins_params.yaml` 已按框架第 7 节预设：640×400 双目、特征点 ≤150、
输出 20 Hz、关闭回环，Kalibr 标定结果直接填入。输出 `/uavN/odom` 转
`vehicle_visual_odometry` 喂给飞控 EKF2 融合（飞控参数 `EKF2_EV_CTRL=15`）。

仿真测试链路（`x500_stereo` 双目模型 + ros_gz_bridge + OpenVINS 仿真配置 +
精度评估脚本）已内置，用法见第 3 节「附加玩法：OpenVINS 双目 VIO 仿真测试」。

### 2.8 uav_bringup —— 启动与参数（总开关）

- `launch/sim_swarm.launch.py`：**仿真一键启动**，拉起 4 个 offboard 节点 +
4 个仿真检测器 + 集群调度 + 防碰监视，`num_uavs`/`sim` 等参数可调
- `launch/goal_nav.launch.py`：**RViz 打点导航**（单机），4 机起飞悬停 +
goal_planner（A*）+ TF/标记桥 + RViz，`uav_id` 选择被控机
- `launch/uav_bringup.launch.py`：实机单机启动（`auto_takeoff` 默认 False，遥控器先验证）
- `config/params.yaml`：**调参唯一入口**——搜索区域、高度层、防碰距离、
仿真目标位置都在这一个文件里
- `config/rm2025.rviz`：RViz 配置（场地地图/规划路径/无人机标记/2D Nav Goal 工具）

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

脚本依次：把 `worlds/rmuc_2025_field.sdf` 复制进 PX4 的 worlds 目录、把
`worlds/models/` 置于 `GZ_SIM_RESOURCE_PATH` 最前（场地网格始终直接加载
工作空间里的 `worlds/models/rmuc_2025/meshes/rmuc_2025.stl` 这一份，不做任何复制），
设置 `PX4_GZ_WORLD=rmuc_2025_field` → 清理上一次仿真的
残留进程（gz-sim 后端常脱离前台进程组，不清会导致新旧 server 串台、界面空白）→
直接启动 gz-server / gz-gui（与 PX4 实例完全解耦）→ 轮询等待 Gazebo 世界就绪后
错峰启动 4 个 standalone PX4 实例（实例内部会无限重试模型创建请求，世界未就绪
不会丢请求）→ 看门狗用世界 pose 信息流逐台确认模型真的出现在世界中（只重启
"进程已退出且模型未出现"的实例，杜绝同名冲突）→ 启动 MicroXRCEAgent
（udp4:8888，自动接管全部 4 个实例）。

Gazebo 窗口中出现的是 **RMUC2025 真实赛场模型**（29.2 m × 16.2 m 全场网格，
源自 [SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)，
已适配 gz-garden 并整体旋转 90° 使长轴沿 NED 北向）：场地两端为红蓝基地，
中央资源岛/能量机关、环形公路、高地等地形均为真实几何。四架 x500 出生在
**蓝方基地启动区（NED 中心约 (10.5, 0)）四周的黄色停机坪**上：

| 无人机 | 公共系 NED 出生点 | Gazebo ENU 出生点 |
| --- | --- | --- |
| uav1 | (9.4, 1.3) | (1.3, 9.4) |
| uav2 | (9.4, -1.3) | (-1.3, 9.4) |
| uav3 | (11.6, 1.3) | (1.3, 11.6) |
| uav4 | (11.6, -1.3) | (-1.3, 11.6) |

> **坐标换算（重要）**：PX4 gz_bridge 按 `NED = (enu_y, enu_x, -enu_z)` 把
> Gazebo ENU 出生点转为各机本地 NED 原点。因此修改出生点时，必须三处同步：
> `scripts/start_sim_4uav.sh` 的 `SPAWN_POSES`（ENU）、`params.yaml` 的
> `spawn_offsets`（NED）、launch 文件中的 `SPAWN_OFFSETS_NED`（NED）。
> >
> 想回到上一版的简化几何场地：`PX4_WORLD=rm2025_field ./scripts/start_sim_4uav.sh`；
> 想用 PX4 空场地：`PX4_WORLD=default ./scripts/start_sim_4uav.sh`。
>
> **关于 rmu_gazebo_simulator 的说明**：该仿真器基于 Ignition Gazebo Fortress
> 且面向地面机器人（rmoss 底盘/云台/发射机构插件），PX4 v1.15.4 的 SITL 需要
> gz-garden，两者不能直接共跑。因此本工作空间采取「取其场地、留我飞控」的方式：
> 仅复用其 RMUC2025 全场网格模型（`worlds/models/rmuc_2025/`，STL + 模型定义，
> 原插件均已移除），世界文件 `worlds/rmuc_2025_field.sdf` 按 PX4 官方模板补齐
> gz-garden 系统插件（NavSat/AirPressure/ApplyLinkWrench 等）。若你需要他们
> 的地面对抗逻辑，可用他们的 Docker 镜像单独跑原仿真器。
>
> **注意**：Git 仓库不含 5.7 MB 的场地网格二进制（zip 发行包已内含）。
> clone 后首次 `./scripts/start_sim_4uav.sh` 会自动运行
> `scripts/fetch_field_model.sh` 下载网格并用 `tools/rasterize_field.py`
> 重新生成 A* 用的占据栅格（需联网 + numpy，可重复执行）。

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
各自飞向分到的条带做割草机搜索（范围 NED x∈[-10,8]，y∈[-6,6]，
覆盖中场至红方半场）→
飞到绿色目标柱（NED (-9, 2)）附近的飞机"发现"目标 →
四机汇聚盘旋 15 秒 → 各自返回出生停机坪上空 → 降落上锁 → 终端打印"任务结束"。

### 第 5 步：结束仿真

终端 A 按 `Ctrl+C`，或任意终端执行 `./scripts/stop_sim.sh`。

### 附加玩法：RViz 打点 + A* 路径规划（单机）

不想跑整套集群任务，只想"点哪飞哪"时，终端 B 换成：

```bash
ros2 launch uav_bringup goal_nav.launch.py              # 打点控制 1 号机
ros2 launch uav_bringup goal_nav.launch.py uav_id:=3    # 打点控制 3 号机
```

启动后 4 机照常自动起飞到分层高度悬停，同时弹出 RViz：
灰色的是赛场占据栅格地图（基地/资源岛/前哨站/高地），彩色方块是 4 架无人机。
用顶部工具栏的 **"2D Nav Goal"** 在地图上按住拖出目标点（和朝向，
朝向目前不用），
`goal_planner` 会在占据栅格上跑 A* 并拉直平滑（绿色线为规划路径），
被控无人机沿路径逐航点飞过去，其余机原地悬停。

- 规划状态：`ros2 topic echo /goal_planner/state`（EXECUTING/REACHED/NO_PATH）
- 打在了障碍上：自动吸附到最近空闲点并打印警告
- **不要与 `run_swarm.sh` 同时跑**——两者都会给 `/uavN/waypoint` 发航点会互抢
- 调地图：`src/uav_planning/uav_planning/field_map.py`（分辨率/膨胀/障碍清单）
- 实机演进：把 field_map 的障碍来源换成 D430i 深度点云局部建图，
  goal_planner 的接口（/goal_pose 进、/uavN/waypoint 出）完全不用动

### 附加玩法：OpenVINS 双目 VIO 仿真测试

在仿真里跑通"双目图像 + IMU → OpenVINS → 里程计 → 与真值对比"的完整链路，
为实机 D430i 无 GPS 定位做参数预演。原理与输入输出见第 2.7 节。

**一次性安装**（只做一次）：

```bash
# 1) gz->ROS2 桥：Humble+Garden 组合的二进制包在 OSRF 源
#    （若已添加过 packages.osrfoundation.org 源直接 apt；否则源码编译
#      ros_gz，编译前 export GZ_VERSION=garden）
sudo apt install ros-humble-ros-gzgarden

# 2) OpenVINS 本体（第三方库，独立工作空间编译，不放进 uav_ws）
mkdir -p ~/catkin_ws_ov/src && cd ~/catkin_ws_ov/src
git clone https://github.com/rpng/open_vins/
cd ~/catkin_ws_ov && colcon build --packages-select ov_core ov_init ov_msckf ov_eval
source ~/catkin_ws_ov/install/setup.bash
# 若编译报 image_transport.h / tf2_geometry_msgs.h deprecated 错误：
# 把 ov_msckf/src/ros/ROS2Visualizer.h 与 ROSVisualizerHelper.h 中
# 对应三个 #include 换成 .hpp 版本即可（上游已知问题）
```

**运行**（三个终端）：

```bash
# 终端 A：VIO 模式起仿真（1 号机自动换 x500_stereo 双目模型，其余机不变）
VIO_UAV=1 ./scripts/start_sim_4uav.sh

# 终端 B：桥接 + OpenVINS（自动读取仿真分区、检查依赖、等相机就绪）
./scripts/run_openvins_sim.sh

# 终端 C：精度评估（OpenVINS 轨迹 vs PX4 SITL 本地位置≈真值，自动对齐后报漂移）
ros2 run uav_localization compare_vio_gt.py --ros-args -p uav_id:=1
```

**操作顺序**：仿真加载完成后先让 1 号机在停机坪**静置约 3 秒**，
终端 B 出现 `Initialized` 再起飞（静止初始化）；之后正常打点或集群飞行，
终端 C 每 5 秒报告一次水平/3D RMSE 与漂移百分比。RViz 可加
`/uav1/trackhist`（特征跟踪图）与 `/uav1/points_msckf`（三角化特征点云）。

**仿真部件与参数对应关系**（改相机参数时三处必须同步）：

| 内容 | 文件 |
|---|---|
| 双目传感器（640×480@30 灰度、基线 50mm、hfov 87°、挂点 base_link (0.17,0,-0.06)） | `worlds/models/stereo_cam/model.sdf` |
| x500 双目变体（merge x500 + 挂 stereo_cam） | `worlds/models/x500_stereo/model.sdf` |
| OpenVINS 估计器参数（特征 150、max_clones 11、静止初始化等） | `src/uav_localization/config/openvins_sim/estimator_config.yaml` |
| IMU 噪声（由 x500_base 的 SDF 噪声换算） | `.../openvins_sim/kalibr_imu_chain.yaml` |
| 相机内外参（fx=fy=337.22、T_imu_cam 与挂点严格对应） | `.../openvins_sim/kalibr_imucam_chain.yaml` |

**排错**：

- **带双目那架无人机直接消失、GUI 里查无此机**：模型 spawn 被 gz-server
  拒绝，按顺序查：
  1. `tail -n 50 /tmp/gz_server.log` 找 `[Err]` 行——最常见的是
     `Camera sensor <image><format> has invalid value of L8_INT8`：
     gz-sim 相机灰度格式的合法名是 **`L_INT8`**（不是 `L8_INT8`），
     非法格式会让服务器拒绝整个模型 spawn（Error Code 9，其余机不受影响）；
     修复：`sed -i 's/L8_INT8/L_INT8/g' worlds/models/stereo_cam/model.sdf`
  2. 若没有格式错误，检查 merge 命名：被 `<include merge='true'>` 拍平进
     x500 的子模型，link 名必须带模型名前缀（`stereo_cam/base_link`，与
     PX4 官方 OakD-Lite 相同写法）；只叫 `base_link` 会与机体重名，
     整个模型加载失败
  3. 确认模型到底生成没有：`source /tmp/rm27_gz_env.sh` 后再
     `gz model --list`（每轮仿真 `GZ_PARTITION` 都不同，不 source 会报
     `Service call to [/gazebo/worlds] timed out`——这不是仿真挂了）
- **场地网格必须是单层版本**：`worlds/models/rmuc_2025/meshes/rmuc_2025.stl`
  应为单层 3m 墙版（md5 `19079837e87223fcc745de9620966053`）。历史上
  存在一份双层重叠版（每个三角形被完整复制一份、22.6 万面，md5
  `fa41fd76...`），会导致接触冲量与渲染负载翻倍，已备份为
  `rmuc_2025_doubled_backup.stl` 留档，**不要再换回去**
- **odomimu 有话题但一条消息都没有（hz 静默）/ OpenVINS 突然不刷日志**：
  仿真重启过而桥接没跟着重启。`start_sim_4uav.sh` 每轮生成新的
  `GZ_PARTITION`，终端 B 的旧桥接还连着已死的分区被无声饿死。
  **铁律：仿真每次重启，`run_openvins_sim.sh` 必须跟着重启**
  （它会自动 source `/tmp/rm27_gz_env.sh` 里的新分区）。同理，
  `ros2 topic list` 里整个 `/px4_*` 消失 = MicroXRCEAgent 没在跑
  （它随仿真脚本自动拉起，仿真重启即恢复）
- **OpenVINS 启动即死 `ParameterAlreadyDeclaredException: topic_imu`**：
  上游 ROS2Visualizer 会自行声明 topic_imu / topic_camera0/1，
  launch 里不得再传同名参数（当前 launch 已移除，勿加回）
- **一直刷 `failed static init: no accel jerk detected`**：配置里
  `try_zupt: false` 导致静止初始化必须等"静止→运动"的 jerk 才触发。
  当前配置已开 ZUPT（仅初始化阶段生效），静置 2 秒即完成初始化；
  若把 try_zupt 关回 false，就得静置后直接起飞，在动起来瞬间初始化
- 桥接报 `Unable to find topic` / 终端 B 一直等相机：仿真是普通模式起的
  （没加 `VIO_UAV=1`），或终端 B 与仿真不在同一 `GZ_PARTITION`
  （`run_openvins_sim.sh` 会自动 source `/tmp/rm27_gz_env.sh`，手动调试时别忘了）
- OpenVINS 终端刷 `cv_bridge exception`：相机像素格式不匹配，stereo_cam
  用的是 `L_INT8`（桥接后为 mono8），改过相机格式的话同步改回
- 一直不初始化：飞机没静置/图像全黑（GUI 里 Topic Visualization 选
  `/vio_cam0/image` 检查画面）；场地纹理少属于正常，CLAHE 已开
- 轨迹飘得离谱：先核对 `kalibr_imucam_chain.yaml` 的 `T_imu_cam` 是否与
  stereo_cam/model.sdf 的挂点一致（两处必须同步改）

## 4. 运行中调试命令

```bash
source /tmp/rm27_gz_env.sh   # 用 gz CLI 前必做：加载本轮仿真分区（每轮重启都会变）
gz model --list              # 已生成的模型列表
gz model -m x500_2 -p        # 某架机的位姿
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
| --- | --- |
| 编译 Agent 报「无效引用：2.12.x」 | v2.4.2 依赖的 Fast-DDS 分支已被官方删除：`git checkout v2.4.3` 后删 `build/` 重新编译（setup_env.sh 已修复） |
| `ros2 topic list` 没有 px4 话题 | Agent 没连上：看终端 A 输出；`tail /tmp/px4_instance_1.log` 查 PX4 日志 |
| 飞机不起飞、卡在 ARMING | 心跳没通：确认 launch 是在 `start_sim_4uav.sh` **之后**启动；检查 offboard 心跳频率是否 ≈10 Hz |
| Gazebo 里飞机数量不够（如只有 1~3 架） | 多为 gz-server 忙时模型创建失败导致 PX4 实例退出：start_sim_4uav.sh 已将 gz-server 与 PX4 解耦——先等世界就绪再错峰启动实例（实例自身无限重试创建请求），看门狗用世界 pose 信息流逐台确认并只重启已退出的实例；仍缺机时看终端 A 的 [sim] 汇总和 /tmp/px4_instance_N.log（勿用"杀掉重启"式脚本：PX4 禁止同名模型，撞名会让实例直接退出） |
| 第二次启动 Gazebo 空白/没有场地 | 上一次仿真没退干净（除 gz-sim 后端外，官方确认 `gz sim -g` 的 ruby 启动器/GUI 也会残留），新旧 server 同时在线导致服务发现串台：start_sim_4uav.sh 启动前按 `gz[- ]sim` 统一匹配强清理，且每轮仿真用唯一 GZ_PARTITION 隔离（残留杀不净也不串台）；手动清理用 `./scripts/stop_sim.sh` 或 `pkill -9 -f "gz[- ]sim"`；仍空白请带上 /tmp/gz_server.log 与 /tmp/gz_gui.log 排查 |
| 改了 params.yaml 没生效 | launch 读的是 install 下的副本：重新 `colcon build` 并 `source install/setup.bash` |
| Gazebo 打开的是空场地而非赛场 | 世界文件没装上：确认终端 A 有「已安装世界文件」输出；否则手动 `cp worlds/rmuc_2025_field.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/`（PX4 的 Tools/simulation/gz 子模块必须已拉取） |
| 场地模型缺失（世界只有几个停机坪/目标柱） | 场地网格没装上：确认 `worlds/models/rmuc_2025/meshes/rmuc_2025.stl` 存在（缺失时脚本会自动下载）；Gazebo 直接加载工作空间这份文件，启动日志会打印其 md5（单层 3m 墙版应为 19079837e87223fcc745de9620966053），更换网格后无需任何同步操作 |
| 飞机出生点与世界对不上（穿模/悬空） | 出生点三处配置不同步：start_sim_4uav.sh 的 SPAWN_POSES、params.yaml 的 spawn_offsets、launch 的 SPAWN_OFFSETS_NED 必须一致（注意 NED=(enu_y, enu_x)） |
| 某机不跟航点 | 确认航点发到了该机的命名空间 `/uavN/waypoint`，且坐标是该机**本地系**（公共系坐标需减出生点偏移） |
| RViz 打开后看不到地图/无人机 | 确认是 `goal_nav.launch.py` 启动的（它才发 `/field_map` 和 `/uav_markers`）；Fixed Frame 必须是 `map`；地图话题 QoS 需 Reliable+Transient Local（rm2025.rviz 已配好） |
| 打点没反应 | `ros2 topic echo /goal_pose` 确认 RViz 发出去了；`ros2 topic echo /goal_planner/state` 看状态；NO_PATH 说明起终点被障碍封死，看 goal_planner 终端日志 |

## 6. 关键约定

1. **坐标系**：全部使用 PX4 本地 NED 系（北 x、东 y、下 z，**高度 = -z**）。
集群公共坐标 = 各机本地坐标 + 出生点偏移（`spawn_offsets`，见第 2 步的出生点换算表）。
2. **命名空间**：上层节点 `/uav1..4`；飞控桥 `/px4_1..4`（实例 i 的 sysid = i+1，已自动换算）。
3. **QoS**：所有 px4_msgs 话题必须 BEST_EFFORT + TRANSIENT_LOCAL（代码内已封装）。
4. **Offboard 心跳**：`OffboardControlMode` + `TrajectorySetpoint` 必须 ≥2 Hz 持续发布。
5. **安全接管**：仿真中 `Ctrl+C` 掉 run_swarm 后飞控会触发失控保护降落；
实机务必保持 ET08 遥控器接管通道有效。

## 7. 迁移实机（RK3566）清单

- [ ] `setup_env.sh` 在 RK3566 上只执行第 1、4、6 步（**不要**装 Gazebo/PX4 SITL）
- [ ] 飞控串口 ↔ uXRCE-DDS Agent：`MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600`
- [ ] 安装 D430i 驱动：`sudo apt install ros-humble-realsense2-camera`，启动命令见
`uav_localization/config/openvins_params.yaml` 头部注释（VIO 模式关深度流）
- [ ] 启动 OpenVINS：`ros2 launch uav_localization openvins.launch.py`，
输出 `/uavN/odom` 转 `vehicle_visual_odometry` 喂给 EKF2（`EKF2_EV_CTRL=15`）
- [ ] 用 `yolo_detector.py` 替换 `sim_target_detector.py`（加载 `models/yolov5s.rknn`；
**D430i 无 RGB，检测用左红外灰度图**）
- [ ] 集群通信改用 WiFi + CycloneDDS
- [ ] 实机首飞用 `uav_bringup.launch.py`（`auto_takeoff:=False`，遥控器接管验证后再放开）
