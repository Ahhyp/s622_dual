cmake_minimum_required(VERSION 3.8)
project(dual_arm_transport_executor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Core: pure C++17, zero ROS dependency.  R1-T13 requires that this package
# pull in no Gazebo / real-driver / production-math dependency, and R1-T01
# requires it to build standalone.
#
# D5-3 exception, deliberate: the collision guard must REUSE the existing
# arm-arm distance model instead of re-deriving the geometry, so the core links
# dual_arm_collision (pure C++/FCL, no ROS).  That is why collision_guard.hpp
# includes dual_arm_collision/dual_arm_collision.hpp.
# ---------------------------------------------------------------------------
find_package(dual_arm_collision REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/state_machine.cpp
  src/precheck.cpp
  src/artifacts.cpp
  src/abort_requester.cpp
  src/hold_requester.cpp
  src/attach_client.cpp
  src/phase_driver.cpp
  src/attach_adapter.cpp
  src/collision_guard.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME}_core PRIVATE
  C43_W3R1_PACKAGE_VERSION="${PROJECT_VERSION}"
)
# PUBLIC: collision_guard.hpp is a public header that includes the collision
# model header, so consumers need its include dirs and its link dependencies.
target_link_libraries(${PROJECT_NAME}_core PUBLIC dual_arm_collision::dual_arm_collision)

# ---------------------------------------------------------------------------
# ROS adapter node (thin).  It owns the communication, the lifecycle and the
# timer; it owns NO decision logic.  Without it the core still builds and all
# W3-R1 tests still run.
# ---------------------------------------------------------------------------
option(C43_WITH_ROS_NODE "Build the rclcpp adapter node (needs dual_arm_qp_controller)" ON)
set(c43_node ON)
if(C43_WITH_ROS_NODE)
  find_package(rclcpp QUIET)
  find_package(std_srvs QUIET)
  find_package(std_msgs QUIET)
  find_package(tf2_msgs QUIET)
  find_package(dual_arm_qp_controller QUIET)
  if(NOT rclcpp_FOUND OR NOT std_srvs_FOUND OR NOT std_msgs_FOUND OR NOT dual_arm_qp_controller_FOUND)
    set(c43_node OFF)
    message(STATUS "dual_arm_transport_executor: ROS adapter node disabled (missing rclcpp/std_srvs/dual_arm_qp_controller)")
  endif()
else()
  set(c43_node OFF)
endif()

if(c43_node)
  # The controller package exports its library target; the generated message
  # headers come with ament_export_include_directories(include).  Linking the
  # exported target is the portable way for a downstream package to get the
  # typesupport symbols without re-generating the interfaces.
  add_executable(c43_s4_executor src/executor_node.cpp src/main.cpp)
  execute_process(COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE C43_GIT_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(NOT C43_GIT_HEAD)
    set(C43_GIT_HEAD "unknown")
  endif()
  target_compile_definitions(c43_s4_executor PRIVATE
    C43_GIT_HEAD="${C43_GIT_HEAD}"
    C43_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
  )
  target_link_libraries(c43_s4_executor PRIVATE
    ${PROJECT_NAME}_core rclcpp::rclcpp ${std_srvs_TARGETS} ${std_msgs_TARGETS} ${tf2_msgs_TARGETS} dual_arm_qp_controller::dual_arm_qp_controller_lib
  )
  install(TARGETS c43_s4_executor RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_state_machine ${PROJECT_NAME}_core)
  ament_add_gtest(test_fault_injection test/test_fault_injection.cpp)
  target_link_libraries(test_fault_injection ${PROJECT_NAME}_core)
  ament_add_gtest(test_artifacts test/test_artifacts.cpp)
  target_link_libraries(test_artifacts ${PROJECT_NAME}_core)
  ament_add_gtest(test_hold_requester test/test_hold_requester.cpp)
  target_link_libraries(test_hold_requester ${PROJECT_NAME}_core)

  ament_add_gtest(test_abort_requester test/test_abort_requester.cpp)
  target_link_libraries(test_abort_requester ${PROJECT_NAME}_core)
  ament_add_gtest(test_phase_driver test/test_phase_driver.cpp)
  target_link_libraries(test_phase_driver ${PROJECT_NAME}_core)

  # W3-R5: fault / safety chain matrix (executor side).
  ament_add_gtest(test_fault_chain test/test_fault_chain.cpp)
  target_link_libraries(test_fault_chain ${PROJECT_NAME}_core)
  ament_add_gtest(test_collision_guard test/test_collision_guard.cpp)
  target_link_libraries(test_collision_guard ${PROJECT_NAME}_core)
  ament_add_gtest(test_attach_adapter test/test_attach_adapter.cpp)
  target_link_libraries(test_attach_adapter ${PROJECT_NAME}_core)
  # W6: Gazebo weld client (startup release / attach snap criterion / detach).
  # C4.3 S4 W7: full-flow wiring (startup guard, GRASP attach, RELEASE detach).
  ament_add_gtest(test_full_flow test/test_full_flow.cpp)
  if(TARGET test_full_flow)
    target_link_libraries(test_full_flow ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_attach test/test_attach.cpp)
  target_link_libraries(test_attach ${PROJECT_NAME}_core)
  ament_add_gtest(test_phase_integration test/test_phase_integration.cpp)
  target_link_libraries(test_phase_integration ${PROJECT_NAME}_core)

  # W3-R6: software dry-run closed loop (no ROS node, no physical plant).
  ament_add_gtest(test_dry_run_loop test/test_dry_run_loop.cpp)
  target_link_libraries(test_dry_run_loop ${PROJECT_NAME}_core)

  # W3-R6: R5 end-to-end hold chain, executor side.  In-process: the real state
  # machine and the real hold requester against a transport-status model that
  # restates the controller's drain/ack semantics (test/r5_hold_chain.hpp).
  ament_add_gtest(test_hold_chain_e2e test/test_hold_chain_e2e.cpp)
  target_link_libraries(test_hold_chain_e2e ${PROJECT_NAME}_core)
  target_compile_definitions(test_artifacts PRIVATE
    C43_W3R1_ARTIFACT_DIR="${CMAKE_CURRENT_BINARY_DIR}/artifacts/w3_r1")
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/artifacts/w3_r1)
  # Determinism + artifact completeness, run as a Python check over generated dirs.
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME w3r1_artifact_schema
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_artifacts.py
            ${CMAKE_CURRENT_BINARY_DIR}/artifacts/w3_r1)
  set_tests_properties(w3r1_artifact_schema PROPERTIES DEPENDS test_artifacts)

  # W4 batch B: acceptance ledger structure/coverage check.  Python + PyYAML
  # only; it reports structure/coverage and never declares an acceptance PASS.
  # The requirement set lives in the checker, not in the ledger it checks.
  add_test(NAME w4a_acceptance_ledger
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_acceptance_yaml.py
            --ledger ${CMAKE_CURRENT_SOURCE_DIR}/config/c43_s4_acceptance.yaml)
  add_test(NAME w4a_acceptance_ledger_negative
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_check_acceptance_negative.py)

  # W3-R2 live integration: a probe *node* (not a gtest) that drives the real
  # core state machine with the status messages that really cross DDS.  The
  # orchestrator is scripts/test_live_phase_integration.py; it is not a ctest
  # entry because it forks `ros2 launch` and needs a private ROS domain.
  if(c43_node)
    add_executable(live_phase_probe test/live_phase_probe.cpp)
    target_link_libraries(live_phase_probe PRIVATE
      ${PROJECT_NAME}_core rclcpp::rclcpp ${std_srvs_TARGETS}
      dual_arm_qp_controller::dual_arm_qp_controller_lib
    )
    install(TARGETS live_phase_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs std_srvs dual_arm_collision)
ament_package()
