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
  ament_add_gtest(test_dual_task_c24 test/test_dual_task_c24.cpp)
  target_link_libraries(test_dual_task_c24 ${PROJECT_NAME})
  ament_add_gtest(test_nullspace_c25 test/test_nullspace_c25.cpp)
  target_link_libraries(test_nullspace_c25 ${PROJECT_NAME})
  ament_add_gtest(test_singularity_c26 test/test_singularity_c26.cpp)
  target_link_libraries(test_singularity_c26 ${PROJECT_NAME})
  find_package(dual_arm_collision QUIET)
  if(dual_arm_collision_FOUND)
    ament_add_gtest(test_cbf_c27 test/test_cbf_c27.cpp)
    target_link_libraries(test_cbf_c27 ${PROJECT_NAME} dual_arm_collision::dual_arm_collision)
  endif()

  ament_add_gtest(test_perf_report_c28 test/test_perf_report_c28.cpp TIMEOUT 300)
  target_link_libraries(test_perf_report_c28 ${PROJECT_NAME})
  if(dual_arm_collision_FOUND)
    target_link_libraries(test_perf_report_c28 dual_arm_collision::dual_arm_collision)
    target_compile_definitions(test_perf_report_c28 PRIVATE DUAL_ARM_HAVE_COLLISION=1)
  endif()
  if(osqp_FOUND)
    ament_add_gtest(test_solver_cross_check test/test_solver_cross_check.cpp)
    target_link_libraries(test_solver_cross_check ${PROJECT_NAME})
    ament_add_gtest(test_qp_benchmark test/test_qp_benchmark.cpp TIMEOUT 300)
    target_link_libraries(test_qp_benchmark ${PROJECT_NAME})
  endif()

  # -------------------------------------------------------------------------
  # C4.3 S4 W2 -- ULP attribution experiment.  READ THE FILE HEADER FIRST: it
  # is NOT the acceptance test and it deliberately enforces no tolerance.
  #
  # The anchored object task is a TEST-ONLY dependency on purpose: writing the
  # object row assembly out again here would be a second copy of the
  # object-frame math, which check_single_object_task_definition.cmake
  # (C4.3 S4 W0-1) explicitly forbids.  The data paths point at the FROZEN
  # artifacts (S3 fixture, S4 trajectory, S2 baseline) so this package keeps no
  # build-time dependency on the controller -- same trick as S4_BASELINE_YAML
  # in dual_arm_qp_controller/CMakeLists.txt.
  # -------------------------------------------------------------------------
  find_package(dual_arm_object_task QUIET)
  if(dual_arm_object_task_FOUND)
    find_package(yaml-cpp REQUIRED)
    ament_add_gtest(test_ulp_attribution test/test_ulp_attribution.cpp TIMEOUT 300)
    target_link_libraries(test_ulp_attribution ${PROJECT_NAME}
      dual_arm_object_task::dual_arm_object_task yaml-cpp)
    target_compile_definitions(test_ulp_attribution PRIVATE
      S4_FIXTURE_CSV="${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_qp_controller/test/fixtures/s3_ff_1_dq.csv"
      S4_TRANSPORT_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_qp_controller/config/c43_s4_transport.yaml"
      S4_BASELINE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_transport/config/transport_baseline.yaml"
      S3_ARTIFACT_CSV="${CMAKE_CURRENT_SOURCE_DIR}/../../runs/c43_s3/w8/ff_1/cycles.csv")
  else()
    message(STATUS "dual_arm_qp_core: dual_arm_object_task not found -> "
                   "test_ulp_attribution (S4 W2 attribution) is NOT built")
  endif()
endif()

# ---------------------------------------------------------------------------
# C4.3 S4 W2-C3 -- D2-B machine gate (approved 2026-10-27, contract v1).
#
# This runs the frozen baseline comparison exactly as an auditor would: over the
# frozen fixtures, with the provenance/library binding, fail-closed.  It is a
# ctest item rather than a gtest because the gate is deliberately implemented as
# a standalone script (so it can be run on a machine without the build tree).
# ---------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  set(_c3fix "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/c3_d2b")
  # The frozen baseline was produced against the INSTALLED library, so the gate
  # grades that copy.  (The build-tree .so has a different rpath and therefore a
  # different sha256; grading it would be a false provenance mismatch.)
  set(_c3lib "${CMAKE_INSTALL_PREFIX}/lib/libdual_arm_qp_core.so")
  if(NOT EXISTS "${_c3lib}")
    set(_c3lib "$<TARGET_FILE:${PROJECT_NAME}>")
  endif()
  message(STATUS "dual_arm_qp_core: c3_d2b_gate grades ${_c3lib}")
  add_test(NAME c3_d2b_gate
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/c3_d2b_gate.py
      --baseline ${_c3fix}/c3_d2b_candidate_baseline.csv
      --rerun    ${_c3fix}/c3_d2b_candidate_baseline.rerun.csv
      --admm     ${_c3fix}/c3_d2b_admm.csv
      --provenance ${_c3fix}/PROVENANCE.md
      --lib ${_c3lib})
  set_tests_properties(c3_d2b_gate PROPERTIES TIMEOUT 180)
else()
  message(STATUS "dual_arm_qp_core: python3 not found -> c3_d2b_gate NOT registered")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 dual_arm_kinematics)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
