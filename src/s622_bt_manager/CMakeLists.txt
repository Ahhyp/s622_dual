cmake_minimum_required(VERSION 3.8)
project(s622_bt_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yolov8_obb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  "action/VisualAlign.action"
  "action/DualMoveToJointState.action" 
  "srv/SetGripper.srv"
  "srv/AttachObject.srv"
  "srv/DetachObject.srv"
  "srv/TransferObject.srv"
  DEPENDENCIES geometry_msgs action_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(bt_executor_node
  src/bt_executor_node.cpp
  src/dummy_nodes.cpp
  src/perception_nodes.cpp
  src/grasp_nodes.cpp
  src/motion_nodes.cpp
  src/gripper_nodes.cpp
  src/servo_nodes.cpp
  src/scene_nodes.cpp
)
target_include_directories(bt_executor_node PRIVATE include)
ament_target_dependencies(bt_executor_node
  rclcpp rclcpp_action std_msgs std_srvs sensor_msgs geometry_msgs
  ament_index_cpp behaviortree_cpp yolov8_obb_msgs
  tf2 tf2_ros tf2_geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


  
target_link_libraries(bt_executor_node "${cpp_typesupport_target}")

install(TARGETS bt_executor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

