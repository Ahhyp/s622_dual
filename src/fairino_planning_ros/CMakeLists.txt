cmake_minimum_required(VERSION 3.8)
project(fairino_planning_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(fairino_planning_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/fairino_planner_manager.cpp
  src/fairino_planning_context.cpp
  src/fairino_ik_plugin.cpp
  src/moveit_collision_checker.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  fairino_planning_core
  tf2_eigen
  geometry_msgs
)

pluginlib_export_plugin_description_file(
  moveit_core
  plugins/fairino_planning_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch plugins
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  fairino_planning_core
)

ament_package()