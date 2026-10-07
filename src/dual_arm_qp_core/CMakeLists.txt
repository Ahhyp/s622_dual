cmake_minimum_required(VERSION 3.16)
project(dual_arm_qp_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dual_arm_kinematics REQUIRED)

# ---------------------------------------------------------------------------
# Solver backend (decision D6).
#   v1: built-in ADMM reference backend (always available, dependency-free).
#   TODO: add the OSQP backend once osqp is installed
#         (e.g. `sudo apt install ros-humble-osqp-vendor`), then implement
#         src/osqp_solver.cpp and enable it here.
# ---------------------------------------------------------------------------
set(QP_SOURCES src/admm_solver.cpp src/dual_arm_qp.cpp)

add_library(${PROJECT_NAME} SHARED ${QP_SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen dual_arm_kinematics::dual_arm_kinematics
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_admm_solver test/test_admm_solver.cpp)
  target_link_libraries(test_admm_solver ${PROJECT_NAME})
  ament_add_gtest(test_dual_arm_qp test/test_dual_arm_qp.cpp)
  target_link_libraries(test_dual_arm_qp ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 dual_arm_kinematics)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
