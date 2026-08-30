cmake_minimum_required(VERSION 3.8)
project(mobile_base_navigation)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY config launch rviz behavior_trees
        DESTINATION share/${PROJECT_NAME})

# Renamed so launch files can say executable='goal_to_plan'.
install(
  PROGRAMS scripts/goal_to_plan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME goal_to_plan
)
install(
  PROGRAMS scripts/estop_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME estop_gate
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME verify_navigation_configuration
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_config.py
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/../mobile_base_description
  )
endif()

ament_package()
