#!/usr/bin/env bash
# =============================================================
# 拉取 RMUC2025 场地网格并重新生成占据栅格地图
#
# 场地模型来源（固定到 commit，保证可复现）：
#   SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator
#   rmu_gazebo_simulator/resource/models/rmuc_2025/meshes/rmuc_2025.stl
#
# Git 仓库不带 5.7 MB 的 STL 二进制（zip 发行包已内含），
# clone 后运行一次本脚本即可；start_sim_4uav.sh 发现缺失时也会自动调用。
#
# raw.githubusercontent.com 在国内经常不可达，因此按顺序尝试多个镜像：
#   1) raw.githubusercontent.com（官方源）
#   2) cdn.jsdelivr.net（国内通常可达，支持 commit 锁定）
#   3) mirror.ghproxy.com / 4) ghproxy.net（公共代理）
# 每个源都校验下载文件大小，防止把错误页/截断文件当成网格。
# =============================================================
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MESH_DIR="$WS_DIR/worlds/models/rmuc_2025/meshes"
STL="$MESH_DIR/rmuc_2025.stl"
REPO_PATH="rmu_gazebo_simulator/resource/models/rmuc_2025/meshes/rmuc_2025.stl"
COMMIT="04e01a3678568667b480662700a88ed920dfcc6b"
EXPECTED_SIZE=5711784     # 原版单面 STL 的字节数
URLS=(
    "https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator/$COMMIT/$REPO_PATH"
    "https://cdn.jsdelivr.net/gh/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator@$COMMIT/$REPO_PATH"
    "https://mirror.ghproxy.com/https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator/$COMMIT/$REPO_PATH"
    "https://ghproxy.net/https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator/$COMMIT/$REPO_PATH"
)

if [ -s "$STL" ]; then
    echo "[field] 场地网格已存在：$STL"
else
    mkdir -p "$MESH_DIR"
    ok=0
    for u in "${URLS[@]}"; do
        echo "[field] 尝试下载（约 5.7 MB）：$u"
        if curl -fL --connect-timeout 10 --retry 2 -o "$STL" "$u" \
            && [ "$(stat -c%s "$STL" 2>/dev/null || echo 0)" = "$EXPECTED_SIZE" ]; then
            ok=1
            echo "[field] 下载完成：$STL（$EXPECTED_SIZE 字节）"
            break
        fi
        echo "[field] 该源失败或文件不完整，换下一个源..."
        rm -f "$STL"
    done
    if [ "$ok" != "1" ]; then
        echo "[field] 错误：所有下载源均失败（网络受限）。"
        echo "[field] 备选：解压 zip 发行包，把 uav_ws/worlds/models/rmuc_2025/meshes/rmuc_2025.stl"
        echo "[field]       复制到 $MESH_DIR/ 后重跑本脚本（zip 内已是双面化修复版）。"
        exit 1
    fi
fi

# 原版网格大量面片法向朝下，Gazebo 背面剔除会导致地板不可见——双面化修复（幂等）
echo "[field] 检查/修复网格朝向（双面化）..."
python3 "$WS_DIR/tools/fix_mesh_normals.py" "$STL"

echo "[field] 重新生成占据栅格地图..."
if ! python3 "$WS_DIR/tools/rasterize_field.py"; then
    echo "[field] 警告：占据栅格生成失败（多半是缺 numpy：sudo apt install python3-numpy）"
    echo "[field] 场地网格已就绪可先用，A* 地图之后重跑本脚本生成即可"
fi
echo "[field] 完成"
