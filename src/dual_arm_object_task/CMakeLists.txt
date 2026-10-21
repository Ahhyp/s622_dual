cmake_minimum_required(VERSION 3.16)
project(dual_arm_object_task)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
# Same flags as dual_arm_transport_mock: the W0 migration is only proven
# numerically inert if both sides are compiled identically.
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(dual_arm_kinematics REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/anchored_object_task.cpp
  src/trajectory.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    yaml-cpp
    dual_arm_kinematics::dual_arm_kinematics
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  # W0-1: exactly one definition of the anchored object task may exist, and the
  # production controller must not reach past the builder to the raw frame math.
  # A comment cannot enforce that; this script can.
  add_test(
    NAME test_single_object_task_definition
    COMMAND ${CMAKE_COMMAND}
            -DSRC_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/..
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_single_object_task_definition.cmake
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp dual_arm_kinematics)
ament_export_include_directories(include)
ament_package()
