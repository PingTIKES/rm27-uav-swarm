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
#   残留的不仅是 gz-sim 后端（常脱离前台进程组），还有名为
#   "gz sim -g"（空格）的 ruby 启动器/GUI 进程——Gazebo 官方文档
#   确认的已知 bug，必须用 gz[- ]sim 同时匹配两种写法才能清干净。
#   旧进程残留时第二次启动会新旧 server 同时在线，GUI/PX4 的服务
#   发现随机各连一个（串台），表现为界面空白或模型不出现。
#   本脚本双保险：
#   1) 启动前/退出时按 gz[- ]sim|px4_sitl|MicroXRCEAgent 强清理
#      （SIGTERM 等待 10 秒，不死再 SIGKILL，仍不死则明确告警）；
#   2) 每轮仿真导出唯一的 GZ_PARTITION（服务发现按分区隔离），
#      即使有杀不掉的残留，新旧两轮也互相不可见。
#   注意：手动在别的终端用 gz topic/gz service 调试时，需先
#   source /tmp/rm27_gz_env.sh（写入本轮分区号）才能看到话题。
#
# gz-server 猝死说明：
#   服务器进程一旦崩溃（如场地网格退化三角形触发 ODE 三角网格
#   碰撞断言，报"已中止 (核心已转储)"），standalone 实例的创建
#   请求会无限重试且 PX4 进程不死，看门狗无法自愈。本脚本在
#   等待世界就绪/错峰启动/看门狗轮询中持续检查服务器存活，
#   一旦猝死立即输出诊断（指向 /tmp/gz_server.log）并整体退出。
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

    # 场地模型（model://rmuc_2025 等）直接由工作空间目录提供：
    # GZ_SIM_RESOURCE_PATH 把它放在最前，Gazebo 无论什么情况都加载
    # worlds/models/rmuc_2025/meshes/rmuc_2025.stl 这一份；
    # 不再复制进 PX4 models 目录（副本会过期，曾导致换网格后仍显示旧版）
    if [ -d "$WS_DIR/worlds/models" ]; then
        # git clone 的工作空间不含 STL 二进制（zip 发行包已内含），缺失时自动拉取
        if [ ! -s "$WS_DIR/worlds/models/rmuc_2025/meshes/rmuc_2025.stl" ] \
            && [ "$WORLD" = "rmuc_2025_field" ]; then
            echo "[sim] 场地网格缺失，尝试自动下载（需联网）..."
            bash "$WS_DIR/scripts/fetch_field_model.sh" || \
                echo "[sim] 警告：场地网格下载失败，Gazebo 中将缺少场地模型"
        fi
        # 删除旧版本脚本复制进 PX4 目录的场地模型副本，杜绝误读旧网格
        if [ -d "$PX4_DIR/Tools/simulation/gz/models/rmuc_2025" ]; then
            rm -rf "$PX4_DIR/Tools/simulation/gz/models/rmuc_2025"
            echo "[sim] 已删除 PX4 models 目录下的旧场地模型副本（统一使用工作空间副本）"
        fi
        export GZ_SIM_RESOURCE_PATH="$WS_DIR/worlds/models:${GZ_SIM_RESOURCE_PATH:-}"
        STL_SRC="$WS_DIR/worlds/models/rmuc_2025/meshes/rmuc_2025.stl"
        if [ -f "$STL_SRC" ]; then
            echo "[sim] 场地模型直接加载：$STL_SRC"
            echo "[sim] 场地网格 md5: $(md5sum "$STL_SRC" | cut -d' ' -f1)"
            echo "[sim]   参考值：fa41fd76e66492d8c87762355f461977（削墙+清理退化三角形版，推荐）"
            echo "[sim]           bf4ab3fc2320af8cff00e6c52be3409d（旧削墙版，含 402 个退化三角形）"
        fi
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
# PX4 自带模型库（x500 等）放在路径最后：工作空间 worlds/models 优先，
# 保证无论什么情况场地模型都加载工作空间里的那一份；
# 本脚本直接启动 gz-server，需显式给出该路径（原来由 gz_env.sh 设置）
if [ -n "${GZ_SIM_RESOURCE_PATH:-}" ]; then
    export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"
else
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models"
fi
# 每轮仿真使用唯一的 Gazebo 传输分区（服务发现按分区隔离）：
# 即使上一次的进程杀不干净，新旧两轮也互相不可见，杜绝串台
export GZ_PARTITION="${GZ_PARTITION:-rm27_$$}"
echo "export GZ_PARTITION=$GZ_PARTITION" > /tmp/rm27_gz_env.sh

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
        if [ -n "${GZ_SERVER_PID:-}" ] && ! kill -0 "$GZ_SERVER_PID" 2>/dev/null; then
            echo "[sim] gz-server 在世界就绪前已退出"
            return 2
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

# 残留进程匹配模式：gz-sim（连字符后端）+ gz sim（空格 ruby 启动器/GUI，
# 官方确认的会残留的进程）+ PX4 + Agent
SIM_PROC_PAT="gz[- ]sim|px4_sitl|MicroXRCEAgent"

# 强清理上一次仿真的全部残留进程
cleanup_sim() {
    pkill -f "px4_sitl" 2>/dev/null || true
    pkill -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -f "gz[- ]sim" 2>/dev/null || true
    for t in $(seq 1 10); do
        pgrep -f "$SIM_PROC_PAT" >/dev/null 2>&1 || return 0
        sleep 1
    done
    echo "[sim] 有残留进程未响应 SIGTERM，强制 SIGKILL..."
    pkill -9 -f "px4_sitl" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -9 -f "gz[- ]sim" 2>/dev/null || true
    sleep 1
    if pgrep -f "$SIM_PROC_PAT" >/dev/null 2>&1; then
        echo "[sim] 警告：以下进程 SIGKILL 仍无法杀死（通常卡在显卡驱动/内核态）："
        pgrep -fa "$SIM_PROC_PAT" || true
        echo "[sim] 本轮已用独立 GZ_PARTITION 隔离可继续运行；若 Gazebo 仍空白只能重启电脑恢复。"
    fi
}

# gz-server 猝死处理：服务器一死，standalone 实例的模型创建请求会无限
# 重试（PX4 进程不死），看门狗再等也是白费——立即给出诊断并整体退出
server_dead_abort() {
    echo "[sim] 严重：gz-server（PID ${GZ_SERVER_PID:-?}）已崩溃退出，后续模型无法加载。"
    echo "[sim] 崩溃原因在 /tmp/gz_server.log 末尾几十行，请执行："
    echo "[sim]   tail -n 50 /tmp/gz_server.log"
    echo "[sim] 并把输出发出来定位（常见为 ODE 三角网格碰撞断言或显卡驱动异常）。"
    kill "${PIDS[@]}" 2>/dev/null || true
    cleanup_sim
    exit 1
}

# ---- 0. 清理上一次仿真的残留进程（首次运行无残留，秒过） ----
if pgrep -f "$SIM_PROC_PAT" >/dev/null 2>&1; then
    echo "[sim] 检测到上一次仿真的残留进程，先清理..."
    cleanup_sim
fi

# ---- 1. 启动 Gazebo（server + GUI），与 PX4 实例完全解耦 ----
echo "[sim] 启动 gz-server（世界 $WORLD，分区 $GZ_PARTITION）..."
gz sim -r -s -v 2 "$WORLD_SDF" > /tmp/gz_server.log 2>&1 &
GZ_SERVER_PID=$!
PIDS+=($GZ_SERVER_PID)
if [ -z "$HEADLESS" ]; then
    gz sim -g -v 2 > /tmp/gz_gui.log 2>&1 &
    PIDS+=($!)
fi

# ---- 2. 等待世界就绪 ----
wr_rc=0
wait_world_ready || wr_rc=$?
[ "$wr_rc" -eq 2 ] && server_dead_abort

# ---- 3. 错峰启动全部 standalone PX4 实例 ----
for i in $(seq 1 "$NUM_UAVS"); do
    kill -0 "$GZ_SERVER_PID" 2>/dev/null || server_dead_abort
    start_uav "$i"
    sleep 5          # 错峰，减轻 gz-server 瞬时压力
done

# ---- 4. 看门狗：确认各机模型加载；只重启"进程已死且模型未出现"的实例 ----
echo "[sim] 看门狗：等待全部 $NUM_UAVS 台模型加载..."
for round in $(seq 1 40); do
    kill -0 "$GZ_SERVER_PID" 2>/dev/null || server_dead_abort
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
echo "[sim] 别的终端手动用 gz topic/gz service 调试前，先执行：source /tmp/rm27_gz_env.sh"
echo "[sim] 如遇界面空白/缺机等异常，请把 /tmp/gz_server.log /tmp/gz_gui.log /tmp/px4_instance_*.log 发出来"

# 清理期间屏蔽再次 Ctrl+C，保证强清理完整执行（被打断会留残余进程）
on_exit() {
    trap '' INT TERM
    echo; echo "[sim] 正在结束仿真..."
    kill "${PIDS[@]}" 2>/dev/null || true
    cleanup_sim
    exit 0
}
trap on_exit INT TERM
wait
