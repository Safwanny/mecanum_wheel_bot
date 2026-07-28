cmake_minimum_required(VERSION 3.8)
project(mobile_base_evaluation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(rclcpp REQUIRED)

add_library(ground_truth_selector_lib
  src/ground_truth_selector.cpp
)
target_include_directories(ground_truth_selector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(ground_truth_selector_lib PUBLIC
  gz-msgs10::gz-msgs10
)

add_executable(ground_truth_selector
  src/ground_truth_selector_node.cpp
)
target_link_libraries(ground_truth_selector
  ground_truth_selector_lib
  gz-transport13::gz-transport13
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

install(
  TARGETS ground_truth_selector ground_truth_selector_lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ground_truth_selector
    test/test_ground_truth_selector.cpp
  )
  target_link_libraries(test_ground_truth_selector
    ground_truth_selector_lib
    gz-msgs10::gz-msgs10
  )
endif()

ament_package()
