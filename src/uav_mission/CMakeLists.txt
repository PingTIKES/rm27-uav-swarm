cmake_minimum_required(VERSION 3.8)
project(uav_mission)

find_package(ament_cmake REQUIRED)

# 安装行为树定义文件
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees)

# 实机 BehaviorTree.CPP 任务节点（取消注释并补充 src/mission_control.cpp 后启用）：
# find_package(behaviortree_cpp REQUIRED)
# add_executable(mission_control src/mission_control.cpp)
# ament_target_dependencies(mission_control rclcpp behaviortree_cpp)
# install(TARGETS mission_control DESTINATION lib/${PROJECT_NAME})

ament_package()
