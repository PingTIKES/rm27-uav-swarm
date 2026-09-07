#!/usr/bin/env bash
# =============================================================
# 启动 4 机 PX4 SITL + Gazebo(RM2025 赛场) + MicroXRCEAgent
#
# 依据 PX4 官方多机仿真方式：
#   - 实例 1 启动 gz-server；其余实例用 PX4_GZ_STANDALONE=1 接入
#   - PX4_GZ_MODEL_POSE 指定各机出生点（Gazebo ENU 坐标）
#   - 单个 MicroXRCEAgent 自动接入全部实例
#   - 话题命名空间自动为 /px4_1 .. /px4_4
#
# 赛场说明（worlds/rmuc_2025_field.sdf）：
#   - RMUC2025 真实赛场网格（源自 SMBU-PolarBear rmu_gazebo_simulator），
#     长轴沿 NED 北向（Gazebo ENU +y）
#   - 蓝方基地在北端（NED x≈+13），其前方 NED (10.5, 0) 为基地启动区
#   - 4 台无人机出生在启动区四周的停机坪上（见 SPAWN_POSES）
#   - 想回到简化几何场地：PX4_WORLD=rm2025_field ./start_sim_4uav.sh
#
# 坐标换算（PX4 gz_bridge）：NED = (enu_y, enu_x, -enu_z)
#   即出生点 ENU "(ex, ey)" 对应公共系 NED "(ey, ex)"，
#   与 params.yaml 中 swarm_coordinator.spawn_offsets 一一对应。
#
# 自定义世界加载原理：PX4 启动脚本固定从
#   $PX4_DIR/Tools/simulation/gz/worlds/<PX4_GZ_WORLD>.sdf 读取世界，
#   因此本脚本会先把 worlds/$WORLD.sdf 复制到该目录。
# =============================================================
set -e
NUM_UAVS=${1:-4}
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
MODEL="${PX4_MODEL:-gz_x500}"        # 可换 gz_x500_depth 等带相机模型
AUTOSTART=4001                        # gz_x500 对应 airframe
WORLD="${PX4_WORLD:-rmuc_2025_field}" # 简化场地：PX4_WORLD=rm2025_field；空场地：default

# 工作空间根目录（本脚本位于 <ws>/scripts/）
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 各机出生点（Gazebo ENU "x,y"，z 留空由 PX4 默认 0.5 m）
# 蓝方基地启动区 NED 中心 (10.5, 0)，四机在其四周：
#   uav1: NED ( 9.4,  1.3) -> ENU "1.3,9.4"
#   uav2: NED ( 9.4, -1.3) -> ENU "-1.3,9.4"
#   uav3: NED (11.6,  1.3) -> ENU "1.3,11.6"
#   uav4: NED (11.6, -1.3) -> ENU "-1.3,11.6"
# （修改后必须同步修改 params.yaml 的 spawn_offsets！）
SPAWN_POSES=("1.3,9.4" "-1.3,9.4" "1.3,11.6" "-1.3,11.6")

if [ ! -x "$PX4_BIN" ]; then
    echo "未找到 PX4 SITL 二进制：$PX4_BIN"
    echo "请先运行 ./setup_env.sh（或 cd \$PX4_DIR && DONT_RUN=1 make px4_sitl_default）"
    exit 1
fi

# 把自定义世界复制进 PX4 的 worlds 目录（PX4 只从该目录加载世界）
if [ "$WORLD" != "default" ]; then
    WORLD_SRC="$WS_DIR/worlds/$WORLD.sdf"
    WORLD_DST="$PX4_DIR/Tools/simulation/gz/worlds/$WORLD.sdf"
    if [ ! -f "$WORLD_SRC" ]; then
        echo "未找到世界文件：$WORLD_SRC"; exit 1
    fi
    if [ ! -d "$(dirname "$WORLD_DST")" ]; then
        echo "未找到 PX4 worlds 目录：$(dirname "$WORLD_DST")"
        echo "请确认 PX4-Autopilot 已完整克隆（含 Tools/simulation/gz 子模块）"; exit 1
    fi
    cp -u "$WORLD_SRC" "$WORLD_DST"
    echo "[sim] 已安装世界文件 -> $WORLD_DST"

    # 同步安装世界引用的网格模型（model://rmuc_2025 等）：
    # 复制进 PX4 的 gz models 目录，并显式加入 GZ_SIM_RESOURCE_PATH
    if [ -d "$WS_DIR/worlds/models" ]; then
        # git clone 的工作空间不含 STL 二进制（zip 发行包已内含），缺失时自动拉取
        if [ ! -s "$WS_DIR/worlds/models/rmuc_2025/meshes/rmuc_2025.stl" ] \
            && [ "$WORLD" = "rmuc_2025_field" ]; then
            echo "[sim] 场地网格缺失，尝试自动下载（需联网）..."
            bash "$WS_DIR/scripts/fetch_field_model.sh" || \
                echo "[sim] 警告：场地网格下载失败，Gazebo 中将缺少场地模型"
        fi
        mkdir -p "$PX4_DIR/Tools/simulation/gz/models"
        # 原版网格法向朝下会被背面剔除（地板不可见），双面化修复是幂等的
        if [ -s "$WS_DIR/worlds/models/rmuc_2025/meshes/rmuc_2025.stl" ]; then
            python3 "$WS_DIR/tools/fix_mesh_normals.py" \
                "$WS_DIR/worlds/models/rmuc_2025/meshes/rmuc_2025.stl" || true
        fi
        cp -ru "$WS_DIR/worlds/models/." "$PX4_DIR/Tools/simulation/gz/models/"
        export GZ_SIM_RESOURCE_PATH="$WS_DIR/worlds/models:$PX4_DIR/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
        echo "[sim] 已安装场地模型 -> $PX4_DIR/Tools/simulation/gz/models/"
        # 离线占据栅格缺失时一并重建（供 goal_planner 的 A* 使用）
        if [ ! -s "$WS_DIR/src/uav_planning/maps/rmuc_2025_occ.npz" ]; then
            python3 "$WS_DIR/tools/rasterize_field.py" || \
                echo "[sim] 警告：占据栅格重建失败，A* 将回退到内置解析障碍"
        fi
        # 若工作空间已编译，同步一份进 install 目录（goal_planner 从
        # share 目录读图；不重新 colcon build 也能生效）
        INSTALL_MAPS="$WS_DIR/install/uav_planning/share/uav_planning/maps"
        if [ -s "$WS_DIR/src/uav_planning/maps/rmuc_2025_occ.npz" ] \
            && [ -d "$WS_DIR/install/uav_planning/share/uav_planning" ]; then
            mkdir -p "$INSTALL_MAPS"
            cp -u "$WS_DIR/src/uav_planning/maps/rmuc_2025_occ.npz" "$INSTALL_MAPS/" 2>/dev/null || true
        fi
    fi
fi
export PX4_GZ_WORLD="$WORLD"

cd "$PX4_DIR"
PIDS=()

for i in $(seq 1 "$NUM_UAVS"); do
    POSE="${SPAWN_POSES[$((i-1))]:-0,0}"
    if [ "$i" -eq 1 ]; then
        echo "[sim] 启动实例 $i（含 gz-server，世界 $WORLD），出生点 ENU($POSE)"
        PX4_SYS_AUTOSTART=$AUTOSTART PX4_SIM_MODEL=$MODEL PX4_GZ_MODEL_POSE="$POSE" \
            "$PX4_BIN" -i "$i" > "/tmp/px4_instance_$i.log" 2>&1 &
    else
        echo "[sim] 启动实例 $i（standalone），出生点 ENU($POSE)"
        PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=$AUTOSTART PX4_SIM_MODEL=$MODEL PX4_GZ_MODEL_POSE="$POSE" \
            "$PX4_BIN" -i "$i" > "/tmp/px4_instance_$i.log" 2>&1 &
    fi
    PIDS+=($!)
    sleep 2                         # 错开启动，避免 Gazebo 模型名竞争
done

sleep 5
echo "[sim] 启动 MicroXRCEAgent（udp4:8888，自动接入全部实例）"
MicroXRCEAgent udp4 -p 8888 &
PIDS+=($!)

echo ""
echo "[sim] 4 机仿真已启动（世界 $WORLD，蓝方基地启动区）。PX4 日志：/tmp/px4_instance_*.log"
echo "[sim] 验证：ros2 topic list | grep px4_"
echo "[sim] 另开终端执行任务：./scripts/run_swarm.sh"
echo "[sim] Ctrl+C 或 ./scripts/stop_sim.sh 结束仿真"

trap 'echo; echo "[sim] 正在结束仿真..."; kill "${PIDS[@]}" 2>/dev/null; pkill -f "px4_sitl" 2>/dev/null; exit 0' INT TERM
wait
