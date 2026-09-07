#!/usr/bin/env bash
# =============================================================
# 拉取 RMUC2025 场地网格并重新生成占据栅格地图
#
# 场地模型来源（固定到 commit，保证可复现）：
#   SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator
#   rmu_gazebo_simulator/resource/models/rmuc_2025/meshes/rmuc_2025.stl
#
# Git 仓库不带 5.7 MB 的 STL 二进制（zip 发行包已内含双面化修复版），
# clone 后运行一次本脚本即可；start_sim_4uav.sh 发现缺失时也会自动调用。
# =============================================================
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MESH_DIR="$WS_DIR/worlds/models/rmuc_2025/meshes"
STL="$MESH_DIR/rmuc_2025.stl"
URL="https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator/04e01a3678568667b480662700a88ed920dfcc6b/rmu_gazebo_simulator/resource/models/rmuc_2025/meshes/rmuc_2025.stl"

if [ -s "$STL" ]; then
    echo "[field] 场地网格已存在：$STL"
else
    echo "[field] 下载 RMUC2025 场地网格（约 5.7 MB）..."
    mkdir -p "$MESH_DIR"
    curl -fL --retry 3 -o "$STL" "$URL"
    echo "[field] 下载完成：$STL"
fi

# 原版网格大量面片法向朝下，Gazebo 背面剔除会导致地板不可见——双面化修复（幂等）
echo "[field] 检查/修复网格朝向（双面化）..."
python3 "$WS_DIR/tools/fix_mesh_normals.py" "$STL"

echo "[field] 重新生成占据栅格地图..."
python3 "$WS_DIR/tools/rasterize_field.py"
echo "[field] 完成"
