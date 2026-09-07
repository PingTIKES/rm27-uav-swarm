#!/usr/bin/env bash
# =============================================================
# 启动 4 机 PX4 SITL + Gazebo(RM2025 赛场) + MicroXRCEAgent
#
# 架构（PX4 官方多机仿真的解耦版）：
#   - 本脚本直接启动 gz-server / gz-gui，再启动 NUM_UAVS 个
#     PX4_GZ_STANDALONE=1 的 PX4 实例接入
#   - standalone 实例内部会无限重试模型创建请求（PX4 源码
#     GZBridge::init 中的 while 循环），世界未就绪不会丢请求
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
# 自定义世界加载：本脚本先把 worlds/$WORLD.sdf 复制到
#   $PX4_DIR/Tools/simulation/gz/worlds/，再用绝对路径启动 gz-server。
#
# 缺机（少飞机）问题说明：
#   PX4 创建模型时 set_allow_renaming(false)：若世界中已存在同名
#   模型，创建请求返回失败，GZBridge 报错后整个 PX4 实例进程退出。
#   因此"探测不到模型就杀掉重启"会造成同名冲突死亡螺旋（曾导致
#   只剩 1 台）。本脚本的做法：
#   1) 先启动 gz-server 并轮询等待世界就绪（/world/<世界>/create
#      服务出现），再错峰启动各 standalone 实例；
#   2) 看门狗用世界 pose 信息流（/world/<世界>/pose/info，世界内
#      真实状态，非 gz model CLI）确认各机模型出现；
#   3) 只有当某实例进程已退出且其模型确实未出现时，才重启该实例
#      （此时世界中无此名字，重启绝无冲突）；进程活着但模型未出现
#      = 创建请求仍在 PX4 内部排队重试，继续等即可。
#
# 第二次启动 Gazebo 空白/缺场地问题说明：
#   gz sim 是启动器，其派生的 gz-sim 后端常脱离前台进程组，Ctrl+C
#   未必能带走。旧 server 残留时第二次启动会新旧两个 server 同时在
#   线，GUI/PX4 的服务发现随机各连一个（串台），表现为界面空白或
#   模型不出现。因此本脚本启动前先强清理上一次的全部残留进程
#   （SIGTERM 等待 10 秒，不死再 SIGKILL），退出时同样强清理。
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

# 把自定义世界复制进 PX4 的 worlds 目录（与 PX4 自带世界同目录管理）
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
        cp -ru "$WS_DIR/worlds/models/." "$PX4_DIR/Tools/simulation/gz/models/"
        export GZ_SIM_RESOURCE_PATH="$WS_DIR/worlds/models:${GZ_SIM_RESOURCE_PATH:-}"
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
# 本脚本直接启动 gz-server，需显式给出 PX4 模型库路径
#（原来由 PX4 启动流程中的 gz_env.sh 设置）
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"

cd "$PX4_DIR"
PIDS=()
declare -A UAV_PID=()                 # 实例号 -> PX4 进程号（看门狗用）
MODEL_BASE="${MODEL#gz_}"             # gz_x500 -> x500（Gazebo 世界中的模型名前缀）
WORLD_SDF="$PX4_DIR/Tools/simulation/gz/worlds/$WORLD.sdf"

# 等待 Gazebo 世界加载完成（/world/<world>/create 服务出现 = 世界就绪，
# 之后 standalone 实例的模型创建请求才会被正常受理）
wait_world_ready() {
    echo "[sim] 等待 Gazebo 世界就绪（/world/$WORLD/create）..."
    for t in $(seq 1 90); do
        if timeout 5 gz service -l 2>/dev/null | grep -q "/world/$WORLD/create"; then
            echo "[sim] Gazebo 世界已就绪（第 $t 次探测）"
            return 0
        fi
        sleep 2
    done
    echo "[sim] 警告：等待世界就绪超时，仍继续启动（standalone 实例会自行重试创建请求）"
    return 1
}

# 用世界 pose 信息流确认某模型真实出现在世界中（一条 Pose_V 消息
# 即包含当前世界全部模型；找不到则 8 秒后超时返回失败）
model_in_world() {
    timeout 8 gz topic -e -t "/world/$WORLD/pose/info" -n 1 2>/dev/null \
        | grep -q "name: \"$1\""
}

start_uav() {
    local i="$1"
    local POSE="${SPAWN_POSES[$((i-1))]:-0,0}"
    echo "[sim] 启动实例 $i（standalone），出生点 ENU($POSE)"
    PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=$AUTOSTART PX4_SIM_MODEL=$MODEL PX4_GZ_MODEL_POSE="$POSE" \
        "$PX4_BIN" -i "$i" > "/tmp/px4_instance_$i.log" 2>&1 &
    UAV_PID[$i]=$!
    PIDS+=($!)
}

# 强清理上一次仿真的全部残留进程：gz-sim 后端常脱离前台进程组，
# 残留的旧 server 会让第二次启动服务发现串台（Gazebo 空白/缺机）
cleanup_sim() {
    pkill -f "px4_sitl" 2>/dev/null || true
    pkill -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "gz-sim" 2>/dev/null || true
    for t in $(seq 1 10); do
        pgrep -f "gz-sim|px4_sitl|MicroXRCEAgent" >/dev/null 2>&1 || return 0
        sleep 1
    done
    echo "[sim] 有残留进程未响应 SIGTERM，强制 SIGKILL..."
    pkill -9 -f "px4_sitl" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -9 -f "gz-sim" 2>/dev/null || true
    sleep 1
}

# ---- 0. 清理上一次仿真的残留进程（首次运行无残留，秒过） ----
if pgrep -f "gz-sim|px4_sitl|MicroXRCEAgent" >/dev/null 2>&1; then
    echo "[sim] 检测到上一次仿真的残留进程，先清理..."
    cleanup_sim
fi

# ---- 1. 启动 Gazebo（server + GUI），与 PX4 实例完全解耦 ----
echo "[sim] 启动 gz-server（世界 $WORLD）..."
gz sim -r -s "$WORLD_SDF" > /tmp/gz_server.log 2>&1 &
PIDS+=($!)
if [ -z "$HEADLESS" ]; then
    gz sim -g > /tmp/gz_gui.log 2>&1 &
    PIDS+=($!)
fi

# ---- 2. 等待世界就绪 ----
wait_world_ready

# ---- 3. 错峰启动全部 standalone PX4 实例 ----
for i in $(seq 1 "$NUM_UAVS"); do
    start_uav "$i"
    sleep 5          # 错峰，减轻 gz-server 瞬时压力
done

# ---- 4. 看门狗：确认各机模型加载；只重启"进程已死且模型未出现"的实例 ----
echo "[sim] 看门狗：等待全部 $NUM_UAVS 台模型加载..."
for round in $(seq 1 40); do
    all_ok=1
    for i in $(seq 1 "$NUM_UAVS"); do
        MN="${MODEL_BASE}_$i"
        if model_in_world "$MN"; then
            continue
        fi
        all_ok=0
        if ! kill -0 "${UAV_PID[$i]}" 2>/dev/null; then
            echo "[sim] 实例 $i 进程已退出且模型 $MN 未出现，重启该实例..."
            start_uav "$i"
        fi
    done
    [ "$all_ok" -eq 1 ] && break
    sleep 3
done

# ---- 5. 汇总各机加载结果 ----
missing=0
for i in $(seq 1 "$NUM_UAVS"); do
    MN="${MODEL_BASE}_$i"
    if model_in_world "$MN"; then
        echo "[sim]   uav$i：模型 $MN 已加载"
    else
        echo "[sim]   uav$i：模型 $MN 未加载（日志 /tmp/px4_instance_$i.log）"
        missing=$((missing+1))
    fi
done

sleep 5
echo "[sim] 启动 MicroXRCEAgent（udp4:8888，自动接入全部实例）"
MicroXRCEAgent udp4 -p 8888 &
PIDS+=($!)

echo ""
if [ "$missing" -eq 0 ]; then
    echo "[sim] $NUM_UAVS 机仿真启动完成，全部模型已确认加载。"
else
    echo "[sim] 仿真启动结束，但有 $missing 台未加载（见上方汇总与各机日志 /tmp/px4_instance_*.log）。"
fi
echo "[sim] 验证：ros2 topic list | grep px4_"
echo "[sim] 另开终端执行任务：./scripts/run_swarm.sh"
echo "[sim] Ctrl+C 或 ./scripts/stop_sim.sh 结束仿真"

trap 'echo; echo "[sim] 正在结束仿真..."; kill "${PIDS[@]}" 2>/dev/null; cleanup_sim; exit 0' INT TERM
wait
