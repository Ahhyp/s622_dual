cmake_minimum_required(VERSION 3.16)
project(dual_arm_qp_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)  # keep parity with fairino_planning_core
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dual_arm_kinematics REQUIRED)
find_package(dual_arm_qp_core REQUIRED)
find_package(dual_arm_collision REQUIRED)
# C4.3 S4 W1: the single definition of the anchored object task + trajectory
# lives in dual_arm_object_task (S4 W0 extraction).  This package reuses it
# rather than re-deriving the anchored / feed-forward math.
find_package(dual_arm_object_task REQUIRED)

# ---- C4.3 S4 W1 interfaces ------------------------------------------------
# NOTE: rosidl_generate_interfaces creates a custom target named ${PROJECT_NAME},
# so the shared library must use a different target name (OUTPUT_NAME keeps the
# plugin file name libdual_arm_qp_controller.so that the plugin xml declares).
# Generated BEFORE the library so the typesupport target exists when linking.
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartTransport.srv"
  "srv/AbortTransport.srv"
  "msg/TransportStatus.msg"
)

add_library(dual_arm_qp_controller_lib SHARED
  src/dual_arm_qp_controller.cpp
  src/transport_task.cpp
  src/timing_recorder.cpp
)
target_include_directories(dual_arm_qp_controller_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dual_arm_qp_controller_lib
  PUBLIC
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${std_srvs_TARGETS}
    realtime_tools::realtime_tools
    Eigen3::Eigen
    dual_arm_kinematics::dual_arm_kinematics
    dual_arm_qp_core::dual_arm_qp_core
    dual_arm_collision::dual_arm_collision
    dual_arm_object_task::dual_arm_object_task
    ${cpp_typesupport_target}
)
set_target_properties(dual_arm_qp_controller_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME ${PROJECT_NAME})

add_executable(dual_arm_qp_monitor src/dual_arm_qp_monitor.cpp)
target_link_libraries(dual_arm_qp_monitor rclcpp::rclcpp sensor_msgs::sensor_msgs_library dual_arm_kinematics::dual_arm_kinematics)

pluginlib_export_plugin_description_file(controller_interface dual_arm_qp_controller_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS dual_arm_qp_controller_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS dual_arm_qp_monitor RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stream_health test/test_stream_health.cpp)
  target_link_libraries(test_stream_health dual_arm_qp_controller_lib)
  ament_add_gtest(test_period_monitor test/test_period_monitor.cpp)
  target_link_libraries(test_period_monitor dual_arm_qp_controller_lib)
  # C4.3 S4 W1: S4-T3 (row/QP parity), S4-T4 (feed-forward), S4-T5 (pi-0.05).
  ament_add_gtest(test_object_row_wiring test/test_object_row_wiring.cpp)
  target_link_libraries(test_object_row_wiring
    dual_arm_qp_controller_lib
    dual_arm_object_task::dual_arm_object_task
    dual_arm_qp_core::dual_arm_qp_core)

  # C4.3 S4 W1 review (A/B/C): protection report honesty, start-pose gate,
  # latch semantics (HOLD-only resume, two failures -> ABORT, FAULT needs reset).
  ament_add_gtest(test_transport_latches test/test_transport_latches.cpp)
  target_link_libraries(test_transport_latches
    dual_arm_qp_controller_lib
    dual_arm_object_task::dual_arm_object_task
    dual_arm_qp_core::dual_arm_qp_core)

  # C4.3 S4 W2: S4-T2 (per-cycle dq must match the frozen S3 ff_1 baseline to
  # <= 1 ULP) and S4-T13 (mock-bench closure + the real E8 executor-heartbeat
  # interlock: never-received, timeout, frozen trajectory clock, re-arm).
  # The bench case forks a `ros2 launch`, so the timeout is generous.
  ament_add_gtest(test_mock_closure test/test_mock_closure.cpp TIMEOUT 900)
  target_link_libraries(test_mock_closure
    dual_arm_qp_controller_lib
    dual_arm_object_task::dual_arm_object_task
    dual_arm_qp_core::dual_arm_qp_core)
  target_compile_definitions(test_mock_closure PRIVATE
    S4_FIXTURE_CSV="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/s3_ff_1_dq.csv"
    S4_TRANSPORT_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/c43_s4_transport.yaml")

  # C4.3 S4 W2 addendum -- the two realtime tests the plan assigns to W2 but
  # that W2 shipped without:
  #   §7 S4-T2  realtime budget over the 638-cycle transport
  #             (P99 <= 6.4 ms, max interval <= 1.5*dt, 0 overruns)  [待标定]
  #   §7 S4-T17 long run, 17 x (lift/lower) + settle = 51.04 s = 6380 cycles:
  #             same bounds plus RSS growth <= 1 MB and no fd growth.
  # Both fork `ros2 launch`, and the long case runs for ~60 s, hence TIMEOUT.
  ament_add_gtest(test_realtime_budget test/test_realtime_budget.cpp TIMEOUT 900)
  target_link_libraries(test_realtime_budget
    dual_arm_qp_controller_lib
    dual_arm_object_task::dual_arm_object_task
    dual_arm_qp_core::dual_arm_qp_core)
  # Task B: characterise the ULP metric and localise the W2 deviation to the
  # trajectory-clock rule rather than to OSQP non-determinism.
  ament_add_gtest(test_ulp_semantics test/test_ulp_semantics.cpp)
  target_link_libraries(test_ulp_semantics
    dual_arm_qp_controller_lib
    dual_arm_object_task::dual_arm_object_task
    dual_arm_qp_core::dual_arm_qp_core)
  target_compile_definitions(test_ulp_semantics PRIVATE
    S4_FIXTURE_CSV="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/s3_ff_1_dq.csv"
    S4_TRANSPORT_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/c43_s4_transport.yaml")

  # Task C: the frozen grasp transforms have one definition and it matches the
  # S2 yaml exactly (the W2 bug was a second, Identity default definition).
  ament_add_gtest(test_frozen_baseline_compat test/test_frozen_baseline_compat.cpp)
  target_link_libraries(test_frozen_baseline_compat
    dual_arm_qp_controller_lib
    dual_arm_object_task::dual_arm_object_task)
  target_compile_definitions(test_frozen_baseline_compat PRIVATE
    S4_BASELINE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../dual_arm_transport/config/transport_baseline.yaml")

  target_compile_definitions(test_realtime_budget PRIVATE
    C43_REALTIME_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/config/c43_s4_realtime_params.yaml"
    C43_TRANSPORT_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/c43_s4_transport.yaml"
    C43_TRANSPORT_LONG_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/c43_s4_transport_long.yaml"
    C43_TIMING_CSV="/tmp/c43s4_timing.csv")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  std_srvs realtime_tools Eigen3 rosidl_default_runtime
  dual_arm_kinematics dual_arm_qp_core dual_arm_collision dual_arm_object_task)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
