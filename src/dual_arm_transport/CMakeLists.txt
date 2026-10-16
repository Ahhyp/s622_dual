cmake_minimum_required(VERSION 3.16)
project(dual_arm_transport)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(dual_arm_kinematics REQUIRED)
find_package(dual_arm_collision REQUIRED)
# Declared explicitly on purpose -- see package.xml and
# C4.3_S2_实施方案.md §10.1 (dual_arm_pose_optimization's public header
# ik_layer.hpp includes fairino_planning_core, but that package does not
# declare it; we close our own closure instead of touching C4.2, per D5).
find_package(fairino_planning_core REQUIRED)
find_package(dual_arm_pose_optimization REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/grasp_config.cpp
  src/ik_enum.cpp
  src/pair_screen.cpp
  src/path_check.cpp
  src/baseline.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         yaml-cpp
         moveit_core::moveit_planning_scene
         moveit_core::moveit_robot_model
         moveit_core::moveit_robot_state
         moveit_core::moveit_collision_detection
         dual_arm_kinematics::dual_arm_kinematics
         dual_arm_collision::dual_arm_collision
         fairino_planning_core::fairino_planning_core
         dual_arm_pose_optimization::dual_arm_pose_optimization
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# W1..W6 driver: freeze the C4.3 transport baseline end to end.
add_executable(c43_s2_baseline src/c43_s2_baseline_main.cpp)
target_link_libraries(c43_s2_baseline ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS c43_s2_baseline RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Shared fixtures, identical to the ones C4.2 uses.
  set(C43_URDF "${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf")
  set(C43_SRDF "${CMAKE_CURRENT_SOURCE_DIR}/../s622_moveit_config/config/s622_dual_arm.srdf")
  set(C43_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/config")

  # S2-T1 .. S2-T4 (+T14): local-frame discipline and gripper geometry (D1/D2)
  ament_add_gtest(test_grasp_config test/test_grasp_config.cpp)
  target_link_libraries(test_grasp_config ${PROJECT_NAME})
  target_compile_definitions(test_grasp_config PRIVATE
    C43_CONFIG_DIR="${C43_CONFIG_DIR}")

  ament_add_gtest(test_local_frame_and_gripper test/test_local_frame_and_gripper.cpp)
  target_link_libraries(test_local_frame_and_gripper ${PROJECT_NAME})
  target_compile_definitions(test_local_frame_and_gripper PRIVATE
    C43_CONFIG_DIR="${C43_CONFIG_DIR}"
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}")

  # S2-T5: branch enumeration completeness
  ament_add_gtest(test_ik_enum test/test_ik_enum.cpp)
  target_link_libraries(test_ik_enum ${PROJECT_NAME})
  target_compile_definitions(test_ik_enum PRIVATE
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}"
    C43_CONFIG_DIR="${C43_CONFIG_DIR}")

  # S2-T6/T8/T10/T11/T12/T13: pair screening, anchors, attribution
  ament_add_gtest(test_pair_screen test/test_pair_screen.cpp)
  target_link_libraries(test_pair_screen ${PROJECT_NAME})
  target_compile_definitions(test_pair_screen PRIVATE
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}"
    C43_CONFIG_DIR="${C43_CONFIG_DIR}")

  # S2-T9/T18: pre-grasp path reachability, independently validated (D7)
  ament_add_gtest(test_path_check test/test_path_check.cpp)
  target_link_libraries(test_path_check ${PROJECT_NAME})
  target_compile_definitions(test_path_check PRIVATE
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}"
    C43_CONFIG_DIR="${C43_CONFIG_DIR}")

  # S2-T7: the R1 defence -- the baseline must NOT be the M4 handover pose
  ament_add_gtest(test_baseline_guard test/test_baseline_guard.cpp)
  target_link_libraries(test_baseline_guard ${PROJECT_NAME})
  target_compile_definitions(test_baseline_guard PRIVATE
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}"
    C43_CONFIG_DIR="${C43_CONFIG_DIR}")

  # S2-T16: the support height comes from screening, never from the candidate
  ament_add_gtest(test_support_and_approach test/test_support_and_approach.cpp)
  target_link_libraries(test_support_and_approach ${PROJECT_NAME})
  target_compile_definitions(test_support_and_approach PRIVATE
    C43_CONFIG_DIR="${C43_CONFIG_DIR}")

  # S2-T17: logSO3 NORM hard filters must be stable near pi (F-S1-1 follow-up)
  ament_add_gtest(test_logso3_norm_stability test/test_logso3_norm_stability.cpp)
  target_link_libraries(test_logso3_norm_stability ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp moveit_core dual_arm_kinematics
                          dual_arm_collision fairino_planning_core
                          dual_arm_pose_optimization)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
