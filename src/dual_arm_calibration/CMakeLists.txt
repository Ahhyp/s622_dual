cmake_minimum_required(VERSION 3.5)
project(dual_arm_calibration)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(dual_arm_calibration)

install(PROGRAMS
  scripts/mc_base_alignment.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_test_pythonpath "${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(test_base_alignment test/test_base_alignment.py
    APPEND_ENV "PYTHONPATH=${_test_pythonpath}")
endif()

ament_package()
