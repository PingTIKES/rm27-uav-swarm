#!/usr/bin/env bash
# =============================================================
# 启动 4 机 PX4 SITL + Gazebo + MicroXRCEAgent
# 依据 PX4 官方多机仿真方式：
#   - 实例 1 启动 gz-server；其余实例用 PX4_GZ_STANDALONE=1 接入
#   - PX4_GZ_MODEL_POSE 错开出生点（y 方向间隔 2 m）
#   - 单个 MicroXRCEAgent 自动接入全部实例
#   - 话题命名空间自动为 /px4_1 .. /px4_4
# =============================================================
set -e
NUM_UAVS=${1:-4}
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
MODEL="${PX4_MODEL:-gz_x500}"     # 可换 gz_x500_depth 等带相机模型
AUTOSTART=4001                     # gz_x500 对应 airframe

if [ ! -x "$PX4_BIN" ]; then
    echo "未找到 PX4 SITL 二进制：$PX4_BIN"
    echo "请先运行 ./setup_env.sh（或 cd \$PX4_DIR && DONT_RUN=1 make px4_sitl_default）"
    exit 1
fi

cd "$PX4_DIR"
PIDS=()

for i in $(seq 1 "$NUM_UAVS"); do
    POSE="0,$(( (i-1) * 2 ))"      # 出生点：东向间隔 2 m
    if [ "$i" -eq 1 ]; then
        echo "[sim] 启动实例 $i（含 gz-server），出生点 $POSE"
        PX4_SYS_AUTOSTART=$AUTOSTART PX4_SIM_MODEL=$MODEL PX4_GZ_MODEL_POSE="$POSE" \
            "$PX4_BIN" -i "$i" > "/tmp/px4_instance_$i.log" 2>&1 &
    else
        echo "[sim] 启动实例 $i（standalone），出生点 $POSE"
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
echo "[sim] 4 机仿真已启动。PX4 日志：/tmp/px4_instance_*.log"
echo "[sim] 验证：ros2 topic list | grep px4_"
echo "[sim] 另开终端执行任务：./scripts/run_swarm.sh"
echo "[sim] Ctrl+C 或 ./scripts/stop_sim.sh 结束仿真"

trap 'echo; echo "[sim] 正在结束仿真..."; kill "${PIDS[@]}" 2>/dev/null; pkill -f "px4_sitl" 2>/dev/null; exit 0' INT TERM
wait
