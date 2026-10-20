cmake_minimum_required(VERSION 3.16)
project(dual_arm_transport_mock)

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
find_package(dual_arm_qp_core REQUIRED)
find_package(dual_arm_transport REQUIRED)
# Declared explicitly on purpose -- see package.xml and
# C4.3_S3_实施方案.md §3.1 (same rationale as C4.3_S2_实施方案.md §10.1).
find_package(fairino_planning_core REQUIRED)
find_package(dual_arm_pose_optimization REQUIRED)

# W1: skeleton.  Sources are appended by W3..W7, one work item at a time.
set(S3_SOURCES
  src/version.cpp
  src/object_task.cpp
  src/trajectory.cpp
  src/transport_checker.cpp
  src/mock_loop.cpp
  src/log_writer.cpp
  src/sha256.cpp
)

# Provenance: the HEAD the binary was built from, so a summary.json can always be
# traced back to a commit even when the working tree has moved on.
execute_process(COMMAND git rev-parse HEAD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_VARIABLE C43_GIT_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET RESULT_VARIABLE C43_GIT_RESULT)
if(NOT C43_GIT_RESULT EQUAL 0 OR C43_GIT_HEAD STREQUAL "")
  set(C43_GIT_HEAD "unknown")
endif()

add_library(${PROJECT_NAME} SHARED ${S3_SOURCES})
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
         dual_arm_qp_core::dual_arm_qp_core
         dual_arm_transport::dual_arm_transport
         fairino_planning_core::fairino_planning_core
         dual_arm_pose_optimization::dual_arm_pose_optimization
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# W7: one artifact directory per run.
add_executable(c43_s3_mock_main src/c43_s3_mock_main.cpp)
target_link_libraries(c43_s3_mock_main ${PROJECT_NAME})
target_compile_definitions(c43_s3_mock_main PRIVATE
  C43_GIT_HEAD="${C43_GIT_HEAD}"
  C43_DEFAULT_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf"
  C43_DEFAULT_SRDF="${CMAKE_CURRENT_SOURCE_DIR}/../s622_moveit_config/config/s622_dual_arm.srdf"
  C43_DEFAULT_S2_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_transport/config"
  C43_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config")
install(TARGETS c43_s3_mock_main RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Shared fixtures, identical to the ones C4.2 / S2 use.
  set(C43_URDF "${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf")
  set(C43_SRDF "${CMAKE_CURRENT_SOURCE_DIR}/../s622_moveit_config/config/s622_dual_arm.srdf")
  # S2 owns the sample/grasp geometry; S3 must consume it, never duplicate it.
  set(C43_S2_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_transport/config")
  # S3's own run config (W7), separate from S2's frozen inputs.
  set(C43_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/config")

  ament_add_gtest(test_version test/test_version.cpp TIMEOUT 600)
  target_link_libraries(test_version ${PROJECT_NAME})

  # W2: QP external object-task row (S3-T5b / S3-T5i).
  ament_add_gtest(test_object_task_row_c43s3 test/test_object_task_row_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_object_task_row_c43s3 ${PROJECT_NAME})

  # W3: anchored object task (S3-T2 / S3-T3 / S3-T8).
  ament_add_gtest(test_object_task_c43s3 test/test_object_task_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_object_task_c43s3 ${PROJECT_NAME})

  # W2b: object-task sign / feed-forward / slack-penalty (S3-T5f / T5g / T5h).
  ament_add_gtest(test_object_task_sign_c43s3 test/test_object_task_sign_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_object_task_sign_c43s3 ${PROJECT_NAME})

  # W4: frozen trajectory + feed-forward (S3-T4).
  ament_add_gtest(test_trajectory_c43s3 test/test_trajectory_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_trajectory_c43s3 ${PROJECT_NAME})
  target_compile_definitions(test_trajectory_c43s3 PRIVATE
    C43_CONFIG_DIR="${C43_CONFIG_DIR}")

  # W5: two-layer transport checker (S3-T9).
  ament_add_gtest(test_transport_checker_c43s3 test/test_transport_checker_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_transport_checker_c43s3 ${PROJECT_NAME})
  target_compile_definitions(test_transport_checker_c43s3 PRIVATE
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}"
    C43_S2_CONFIG_DIR="${C43_S2_CONFIG_DIR}")

  # W6: two-phase closed loop + log writer (S3-T6 / T7 / T19 / T20).
  ament_add_gtest(test_mock_loop_c43s3 test/test_mock_loop_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_mock_loop_c43s3 ${PROJECT_NAME})
  target_compile_definitions(test_mock_loop_c43s3 PRIVATE
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}"
    C43_S2_CONFIG_DIR="${C43_S2_CONFIG_DIR}" C43_CONFIG_DIR="${C43_CONFIG_DIR}")

  # W6: the failure policy (S3-T13) and the rank precheck (S3-T18).
  ament_add_gtest(test_failure_policy_c43s3 test/test_failure_policy_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_failure_policy_c43s3 ${PROJECT_NAME})
  target_compile_definitions(test_failure_policy_c43s3 PRIVATE
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}" C43_CONFIG_DIR="${C43_CONFIG_DIR}"
    C43_S2_CONFIG_DIR="${C43_S2_CONFIG_DIR}")

  # W9: the near-pi contrast (S3-T10 / S3-T11) and the R24 static guard (S3-T12).
  ament_add_gtest(test_near_pi_c43s3 test/test_near_pi_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_near_pi_c43s3 ${PROJECT_NAME})
  target_include_directories(test_near_pi_c43s3 PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/tools")

  # S3-T12 / R24 -- the static guard.  The anchored object row replaces the legacy
  # midpoint API; if the S3 library or headers reach for the legacy entry points, the
  # whole point of S3 (no pi pathology, no symmetric midpoint) is quietly lost.  A
  # grep at configure time is crude, which is exactly why it works: it cannot be
  # bypassed by a clever call path.
  file(GLOB_RECURSE C43_S3_GUARDED_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")
  foreach(forbidden IN ITEMS worldTobject objectJacobianWorld objectJacobianBody
                            objectError objectErrorJacobian)
    foreach(f IN LISTS C43_S3_GUARDED_SOURCES)
      file(STRINGS "${f}" hits REGEX "${forbidden}")
      if(hits)
        string(REPLACE ";" "\n  " joined "${hits}")
        message(FATAL_ERROR
          "S3-T12/R24 violated: '${forbidden}' appears in ${f}\n  ${joined}\n"
          "S3 must use the anchored row (anchoredObjectPose / anchoredObjectErrorJacobian); "
          "the legacy midpoint object API is what S3 exists to replace.")
      endif()
    endforeach()
  endforeach()

  # W7: artifacts + provenance + determinism (S3-T14).
  ament_add_gtest(test_artifacts_c43s3 test/test_artifacts_c43s3.cpp TIMEOUT 600)
  # S3-T12 / R24 static guard.  A plain cmake -P script, so it adds no dependency and
  # runs in the same ctest sweep as the rest of the suite.
  add_test(NAME test_no_legacy_object_api_c43s3
           COMMAND ${CMAKE_COMMAND}
                   -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                   -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_legacy_object_api.cmake)
  target_link_libraries(test_artifacts_c43s3 ${PROJECT_NAME})
  target_compile_definitions(test_artifacts_c43s3 PRIVATE
    C43_URDF="${C43_URDF}" C43_SRDF="${C43_SRDF}"
    C43_S2_CONFIG_DIR="${C43_S2_CONFIG_DIR}" C43_CONFIG_DIR="${C43_CONFIG_DIR}"
    C43_CLI="$<TARGET_FILE:c43_s3_mock_main>")

  # W2: legacy behaviour freeze (S3-T5c).  tools/ carries the shared case table,
  # test/ carries the generated golden dq values.
  ament_add_gtest(test_legacy_golden_c43s3 test/test_legacy_golden_c43s3.cpp TIMEOUT 600)
  target_link_libraries(test_legacy_golden_c43s3 ${PROJECT_NAME})
  target_include_directories(test_legacy_golden_c43s3 PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/tools"
    "${CMAKE_CURRENT_SOURCE_DIR}/test")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp moveit_core dual_arm_kinematics
                          dual_arm_collision dual_arm_qp_core dual_arm_transport
                          fairino_planning_core dual_arm_pose_optimization)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
