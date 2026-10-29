cmake_minimum_required(VERSION 3.22)
project(qrb_ros_slam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/CommandCode.msg"
  "msg/CommandResponse.msg"
  "msg/StatusCode.msg"
  "srv/SlamCommand.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
