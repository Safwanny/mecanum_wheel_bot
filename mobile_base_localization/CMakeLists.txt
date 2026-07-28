cmake_minimum_required(VERSION 3.8)
project(mobile_base_localization)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME verify_ekf_configuration
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ekf_config.py
      ${CMAKE_CURRENT_SOURCE_DIR}/config/ekf.yaml
  )
endif()

ament_package()
