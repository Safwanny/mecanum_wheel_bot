cmake_minimum_required(VERSION 3.8)
project(mobile_base_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(plugin_dependencies
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# The alternate motion profile: an eight-heading lattice planner and a
# controller that drives one primitive at a time. Both are opt-in plugins;
# SmacPlanner2D and MPPI remain the default stack and are untouched.
add_library(mobile_base_navigation_plugins SHARED
  src/lattice_search.cpp
  src/lattice_planner.cpp
  src/primitive_motion_core.cpp
  src/primitive_controller.cpp
)
target_include_directories(mobile_base_navigation_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mobile_base_navigation_plugins ${plugin_dependencies})

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(
  TARGETS mobile_base_navigation_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

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
  PROGRAMS scripts/nav_autostart.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav_autostart
)

ament_export_include_directories(include)
ament_export_libraries(mobile_base_navigation_plugins)
ament_export_dependencies(${plugin_dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME verify_navigation_configuration
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_config.py
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/../mobile_base_description
  )

  # Edge generation, heading snapping and the merge, against a synthetic
  # costmap. No node, no lifecycle, no simulator.
  ament_add_gtest(test_lattice_search test/test_lattice_search.cpp)
  target_link_libraries(test_lattice_search mobile_base_navigation_plugins)
  ament_target_dependencies(test_lattice_search nav2_costmap_2d)

  # The one-primitive-at-a-time invariant, asserted on every tick of a driven
  # state machine rather than sampled from a running robot.
  ament_add_gtest(test_primitive_motion_core test/test_primitive_motion_core.cpp)
  target_link_libraries(test_primitive_motion_core mobile_base_navigation_plugins)

  # Plans a real goal against the saved navigation_basic map through
  # planner_server, and checks the previewed /plan is axis-snapped end to end.
  # This is the regression check on the motion constraints actually reaching
  # the path, which is where the previous attempt at this feature failed.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_lattice_plan_preview.py
    TIMEOUT 180
  )
  set_tests_properties(
    test_test_lattice_plan_preview.py
    PROPERTIES
      ENVIRONMENT
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=33;GZ_PARTITION=mobile_base_lattice_test"
      RUN_SERIAL TRUE
  )
endif()

ament_package()
