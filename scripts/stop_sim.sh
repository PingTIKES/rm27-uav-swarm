#!/usr/bin/env bash
# 结束全部仿真进程。匹配 gz[- ]sim 同时覆盖连字符后端（gz-sim-server，
# 常脱离前台进程组）与空格的 ruby 启动器/GUI（gz sim -g，官方确认
# 会残留的进程）；SIGTERM 后仍未退出的 SIGKILL 兜底，
# 避免残留导致下次启动 Gazebo 空白。
pkill -f "px4_sitl" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "gz[- ]sim" 2>/dev/null
sleep 2
pkill -9 -f "px4_sitl" 2>/dev/null
pkill -9 -f "MicroXRCEAgent" 2>/dev/null
pkill -9 -f "gz[- ]sim" 2>/dev/null
echo "[sim] 已结束全部仿真进程"
