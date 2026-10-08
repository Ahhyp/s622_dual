cmake_minimum_required(VERSION 3.16)
project(dual_arm_qp_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)  # keep parity with fairino_planning_core
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dual_arm_kinematics REQUIRED)

# ---------------------------------------------------------------------------
# Solver backend (decision D6):
#   - ADMM reference backend: always built (dependency-free)
#   - OSQP backend: built when the osqp package is found
#     (install with: sudo apt install ros-humble-osqp-vendor)
# ---------------------------------------------------------------------------
find_package(osqp QUIET)
set(QP_SOURCES src/admm_solver.cpp src/dual_arm_qp.cpp)
if(osqp_FOUND)
  list(APPEND QP_SOURCES src/osqp_solver.cpp)
  message(STATUS "dual_arm_qp_core: OSQP found -> OSQP backend enabled")
else()
  message(STATUS "dual_arm_qp_core: OSQP not found -> ADMM backend only")
endif()

add_library(${PROJECT_NAME} SHARED ${QP_SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen dual_arm_kinematics::dual_arm_kinematics
)
if(osqp_FOUND)
  # OSQP is an implementation detail (only used inside osqp_solver.cpp), so it
  # is a PRIVATE dependency and does not leak to downstream packages.
  target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp)
  target_compile_definitions(${PROJECT_NAME} PRIVATE DUAL_ARM_QP_HAVE_OSQP=1)
endif()
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
  ament_add_gtest(test_closed_loop_c23 test/test_closed_loop_c23.cpp)
  target_link_libraries(test_closed_loop_c23 ${PROJECT_NAME})
  if(osqp_FOUND)
    ament_add_gtest(test_solver_cross_check test/test_solver_cross_check.cpp)
    target_link_libraries(test_solver_cross_check ${PROJECT_NAME})
    ament_add_gtest(test_qp_benchmark test/test_qp_benchmark.cpp TIMEOUT 300)
    target_link_libraries(test_qp_benchmark ${PROJECT_NAME})
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 dual_arm_kinematics)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
