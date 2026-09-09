cmake_minimum_required(VERSION 3.8)
project(uav_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# VIO 评估等 python 工具（ros2 run uav_localization <脚本名>）
install(PROGRAMS
  scripts/compare_vio_gt.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
