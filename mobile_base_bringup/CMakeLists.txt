cmake_minimum_required(VERSION 3.8)
project(mobile_base_bringup)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/generate_sim_sdf.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME verify_teleop_trajectory_configuration
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_teleop_trajectory_config.py
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/../mobile_base_description
      ${CMAKE_CURRENT_SOURCE_DIR}/../mobile_base_gazebo
  )

  add_test(
    NAME verify_mecanum_wheel_contact
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mecanum_wheel_contact.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_launch_test(
    test/test_sensor_topics.py
    TIMEOUT 150
  )
  add_launch_test(
    test/test_odometry_evaluation_runtime.py
    TIMEOUT 120
  )
  set_tests_properties(
    test_test_sensor_topics.py
    PROPERTIES
      ENVIRONMENT
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=31;GZ_PARTITION=mobile_base_sensor_test"
      RUN_SERIAL TRUE
  )
  set_tests_properties(
    test_test_odometry_evaluation_runtime.py
    PROPERTIES
      ENVIRONMENT
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=32;GZ_PARTITION=mobile_base_odometry_test"
      RUN_SERIAL TRUE
  )
endif()

ament_package()
