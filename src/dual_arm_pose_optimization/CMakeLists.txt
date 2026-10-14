cmake_minimum_required(VERSION 3.16)
project(dual_arm_pose_optimization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dual_arm_kinematics REQUIRED)
find_package(dual_arm_collision REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/scene_loader.cpp
  src/collision_backend.cpp
  src/ik_layer.cpp
  src/m4_baseline.cpp
  src/candidate_pool.cpp
  src/feasibility.cpp
  src/optimizer.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         moveit_core::moveit_planning_scene
         moveit_core::moveit_robot_model
         moveit_core::moveit_robot_state
         moveit_core::moveit_collision_detection
         moveit_core::moveit_collision_detection_fcl
         moveit_core::moveit_kinematics_base
         urdf::urdf
         srdfdom::srdfdom
         ament_index_cpp::ament_index_cpp
         dual_arm_kinematics::dual_arm_kinematics
         dual_arm_collision::dual_arm_collision
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# S1 acceptance probe: reports loaded geometry, ACM behaviour, distance scopes.
add_executable(c42_scene_probe src/c42_scene_probe_main.cpp)
target_link_libraries(c42_scene_probe ${PROJECT_NAME})

# S2: IK layer + official Baseline B rebuild.
add_executable(c42_baseline_probe src/c42_baseline_probe_main.cpp)
target_link_libraries(c42_baseline_probe ${PROJECT_NAME})

# S3: layered reachability sweep S0 -> S4.
add_executable(c42_reachability src/c42_reachability_main.cpp)
target_link_libraries(c42_reachability ${PROJECT_NAME})

# S5: V0/V1 pose refinement + standby search.
add_executable(c42_optimize_pose src/c42_optimize_pose_main.cpp)
target_link_libraries(c42_optimize_pose ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS c42_scene_probe c42_baseline_probe c42_reachability c42_optimize_pose
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The dual-arm URDF fixture lives in dual_arm_kinematics; regenerate with
  #   src/dual_arm_kinematics/scripts/regen_fixture.sh
  set(C42_URDF "${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf")
  set(C42_SRDF "${CMAKE_CURRENT_SOURCE_DIR}/../s622_moveit_config/config/s622_dual_arm.srdf")

  ament_add_gtest(test_scene_loader test/test_scene_loader.cpp)
  target_link_libraries(test_scene_loader ${PROJECT_NAME})
  target_compile_definitions(test_scene_loader PRIVATE
    C42_URDF="${C42_URDF}" C42_SRDF="${C42_SRDF}"
    C42_FIXTURE_DIR="${CMAKE_CURRENT_BINARY_DIR}/fixtures")

  ament_add_gtest(test_collision_backend test/test_collision_backend.cpp)
  target_link_libraries(test_collision_backend ${PROJECT_NAME})
  target_compile_definitions(test_collision_backend PRIVATE
    C42_URDF="${C42_URDF}" C42_SRDF="${C42_SRDF}")

  ament_add_gtest(test_ik_layer test/test_ik_layer.cpp)
  target_link_libraries(test_ik_layer ${PROJECT_NAME})

  ament_add_gtest(test_optimizer test/test_optimizer.cpp)
  target_link_libraries(test_optimizer ${PROJECT_NAME})
  target_compile_definitions(test_optimizer PRIVATE
    C42_URDF="${C42_URDF}" C42_SRDF="${C42_SRDF}")

  ament_add_gtest(test_feasibility test/test_feasibility.cpp)
  target_link_libraries(test_feasibility ${PROJECT_NAME})
  target_compile_definitions(test_feasibility PRIVATE
    C42_URDF="${C42_URDF}" C42_SRDF="${C42_SRDF}")

  ament_add_gtest(test_m4_baseline test/test_m4_baseline.cpp)
  target_link_libraries(test_m4_baseline ${PROJECT_NAME})
  target_compile_definitions(test_m4_baseline PRIVATE
    C42_URDF="${C42_URDF}" C42_SRDF="${C42_SRDF}")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 moveit_core urdf srdfdom ament_index_cpp
                          dual_arm_kinematics dual_arm_collision)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
