cmake_minimum_required(VERSION 3.16)
project(dual_arm_kinematics)

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
find_package(fairino_planning_core REQUIRED)

add_library(${PROJECT_NAME} SHARED src/dual_arm_kinematics.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen fairino_planning_core::fairino_planning_core
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
  find_package(kdl_parser REQUIRED)
  find_package(orocos_kdl REQUIRED)

  ament_add_gtest(test_step0_fk_oracle test/test_step0_fk_oracle.cpp)
  target_link_libraries(test_step0_fk_oracle
    ${PROJECT_NAME} kdl_parser::kdl_parser ${orocos_kdl_LIBRARIES} tinyxml2)
  target_include_directories(test_step0_fk_oracle PRIVATE ${orocos_kdl_INCLUDE_DIRS})
  target_compile_definitions(test_step0_fk_oracle PRIVATE
    DUAL_ARM_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/s622_dual_arm.urdf")

  ament_add_gtest(test_step0_relative test/test_step0_relative.cpp)
  target_link_libraries(test_step0_relative ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 fairino_planning_core)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
