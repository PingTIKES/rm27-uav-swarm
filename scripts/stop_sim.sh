#!/usr/bin/env bash
# 结束全部仿真进程
pkill -f "px4_sitl" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
echo "[sim] 已结束全部仿真进程"
