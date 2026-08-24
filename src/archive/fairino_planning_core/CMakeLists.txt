cmake_minimum_required(VERSION 3.8)
project(fairino_planning_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------- find dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                # ← 新增
find_package(geometry_msgs REQUIRED)         # ← 新增
find_package(sensor_msgs REQUIRED)           # ← 新增
find_package(tf2 REQUIRED)                   # ← 新增（如果用 tf2 转 pose）
find_package(tf2_eigen REQUIRED)             # ← 新增
find_package(Eigen3 REQUIRED)
find_package(fairino_msgs REQUIRED)

# ---------------- nanoflann (保留原逻辑) ----------------
include(FetchContent)

find_path(NANOFLANN_INCLUDE_DIR
  NAMES nanoflann.hpp
  HINTS /usr/include /usr/local/include
)

if(NOT NANOFLANN_INCLUDE_DIR)
  message(STATUS "nanoflann not found locally. Fetching nanoflann...")
  FetchContent_Declare(
    nanoflann
    GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(nanoflann)
  set(NANOFLANN_INCLUDE_DIR ${nanoflann_SOURCE_DIR}/include)
endif()

# ---------------- 核心 library ----------------
add_library(${PROJECT_NAME} SHARED
  src/dh_kinematics.cpp
  src/ik/fairino_ik.cpp
  src/ik/ik_selector.cpp
  src/tree/rrt_tree.cpp
  src/algorithms/bi_rrt_star.cpp
  src/service/fairino_ik_service.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${NANOFLANN_INCLUDE_DIR}
)

# 用 ament_target_dependencies 自动处理 include + link
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_eigen
  fairino_msgs
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# ---------------- 可执行文件 1: test_core ----------------
add_executable(test_core
  src/test_core.cpp
)
target_link_libraries(test_core ${PROJECT_NAME})
target_include_directories(test_core PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

# ---------------- 可执行文件 2: service node ----------------
add_executable(fairino_ik_service_node
  src/nodes/fairino_ik_service_node.cpp
)
target_link_libraries(fairino_ik_service_node ${PROJECT_NAME})
ament_target_dependencies(fairino_ik_service_node
  rclcpp
  fairino_msgs
)

# ---------------- install ----------------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS test_core fairino_ik_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# ---------------- export ----------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_eigen
  fairino_msgs
  Eigen3
)

ament_package()
