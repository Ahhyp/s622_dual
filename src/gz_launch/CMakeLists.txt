cmake_minimum_required(VERSION 3.8)
project(gz_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(C43_IGN REQUIRED IMPORTED_TARGET ignition-transport11 ignition-msgs8)
add_executable(c43_dual_force scripts/c43_dual_force.cpp)
target_link_libraries(c43_dual_force PkgConfig::C43_IGN)
install(TARGETS c43_dual_force RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(DIRECTORY scripts launch config rviz worlds models
  DESTINATION share/${PROJECT_NAME}/ )

install(
  PROGRAMS
    scripts/demo_pathplanning_node.py
    scripts/move_to_calibration_pose.py
    scripts/auto_collect_eye_on_base.py
    scripts/c43_homing.py
    scripts/c43_run_contact_transport.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
