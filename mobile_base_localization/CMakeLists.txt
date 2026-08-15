cmake_minimum_required(VERSION 3.8)
project(mobile_base_localization)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

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

  add_test(
    NAME verify_phase2_configuration
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_phase2_config.py
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/../mobile_base_bringup
  )
endif()

ament_package()
