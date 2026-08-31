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
install(
  PROGRAMS scripts/staged_pose_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME staged_pose_controller
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Spins real nodes on the ROS graph, so it needs its own domain and must
  # not run beside anything else publishing the same topics. 31 and 32 are
  # taken by the bringup Gazebo tests.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_staged_pose_controller
    test/test_staged_pose_controller.py
    TIMEOUT 180
  )
  set_tests_properties(
    test_staged_pose_controller
    PROPERTIES
      ENVIRONMENT
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=33"
      RUN_SERIAL TRUE
  )

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
