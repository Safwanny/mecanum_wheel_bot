cmake_minimum_required(VERSION 3.8)
project(mobile_base_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME generate_mobile_base_urdf
    COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/mobile_base.urdf.xacro
  )

  add_test(
    NAME verify_mecanum_wheels
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mecanum_wheels.py
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/mobile_base.urdf.xacro
  )

  add_test(
    NAME verify_sensor_description
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sensor_description.py
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/mobile_base.urdf.xacro
  )
endif()

ament_package()
