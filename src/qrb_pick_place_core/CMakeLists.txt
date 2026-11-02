cmake_minimum_required(VERSION 3.22)
project(qrb_pick_place_core)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_pro_controllers_msgs REQUIRED)
find_package(moveit_pro_base REQUIRED)
find_package(moveit_pro_planning_scene_monitor REQUIRED)
find_package(pro_rrt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(motion_planner SHARED src/motion_planner.cpp)
target_include_directories(motion_planner PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(
  motion_planner
  PUBLIC moveit_pro_base::moveit_pro_base_robot_model
         moveit_pro_base::moveit_pro_base_robot_state
         ${trajectory_msgs_TARGETS}
  PRIVATE moveit_pro_base::moveit_pro_base_planning_scene pro_rrt::pro_rrt)

add_executable(qrb_ros_arm_pick_place src/qrb_ros_arm_pick_place.cpp)
target_link_libraries(
  qrb_ros_arm_pick_place
  motion_planner
  moveit_pro_planning_scene_monitor::planning_scene_monitor
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${control_msgs_TARGETS}
  ${moveit_pro_controllers_msgs_TARGETS})

install(TARGETS motion_planner LIBRARY DESTINATION lib)
install(TARGETS qrb_ros_arm_pick_place DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion_planner test/test_motion_planner.cpp TIMEOUT 120)
  target_compile_definitions(test_motion_planner PRIVATE TEST_FIXTURES_DIR="${PROJECT_SOURCE_DIR}/test/fixtures")
  target_link_libraries(test_motion_planner motion_planner)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node_startup_failure test/test_node_startup_failure.py TIMEOUT 90)
  ament_add_pytest_test(test_node_cancels_on_timeout test/test_node_cancels_on_timeout.py TIMEOUT 120)
endif()

ament_package()
