cmake_minimum_required(VERSION 3.16)
project(dual_arm_workspace_analysis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dual_arm_kinematics REQUIRED)
find_package(dual_arm_collision REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED src/workspace_sampler.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         dual_arm_kinematics::dual_arm_kinematics
         dual_arm_collision::dual_arm_collision
         Threads::Threads
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Sampler executable (deterministic; not a ROS node, but installed where
# `ros2 run dual_arm_workspace_analysis workspace_sample` finds it).
add_executable(workspace_sample src/workspace_sample_main.cpp)
target_link_libraries(workspace_sample ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS workspace_sample
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sampler test/test_sampler.cpp)
  target_link_libraries(test_sampler ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 dual_arm_kinematics dual_arm_collision)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
