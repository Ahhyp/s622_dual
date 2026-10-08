cmake_minimum_required(VERSION 3.16)
project(dual_arm_collision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dual_arm_kinematics REQUIRED)
find_package(fcl REQUIRED)

add_library(${PROJECT_NAME} SHARED src/dual_arm_collision.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen dual_arm_kinematics::dual_arm_kinematics fcl
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_collision test/test_collision.cpp)
  target_link_libraries(test_collision ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 dual_arm_kinematics fcl)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
