cmake_minimum_required(VERSION 3.22)
project(qrb_remote_assistant_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

install(PROGRAMS scripts/slam_command_shim.py scripts/detection_bridge.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/nav2_on_wall_time/ros2 DESTINATION share/${PROJECT_NAME}/scripts/nav2_on_wall_time)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_slam_command_shim test/test_slam_command_shim.py TIMEOUT 120)
  ament_add_pytest_test(test_detection_bridge test/test_detection_bridge.py TIMEOUT 60)
endif()

ament_package()
