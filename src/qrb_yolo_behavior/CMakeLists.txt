cmake_minimum_required(VERSION 3.22)

project(qrb_yolo_behavior)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_pro_behavior
  moveit_pro_behavior_interface
  pluginlib
  rclcpp
  sensor_msgs
  vision_msgs
)

foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  qrb_yolo_behavior
  SHARED
  src/qrb_yolo_behavior.cpp
  src/register_behaviors.cpp
)

target_include_directories(
  qrb_yolo_behavior
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  qrb_yolo_behavior
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  TARGETS qrb_yolo_behavior
  EXPORT qrb_yolo_behaviorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  moveit_pro_behavior_test(qrb_yolo_behavior)
endif()

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface
  qrb_yolo_behavior_plugin_description.xml
)

ament_export_targets(
  qrb_yolo_behaviorTargets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
