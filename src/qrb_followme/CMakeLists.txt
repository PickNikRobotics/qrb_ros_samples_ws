cmake_minimum_required(VERSION 3.16)
project(qrb_followme)

#
# Compiler settings
#
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# Dependencies
#
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_pro_behavior_interface REQUIRED)
find_package(moveit_studio_internal_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

#
# Behavior plugin library
#
# This target generates:
#   libqrb_target_tracking.so
#
# It must match:
#   <library path="qrb_target_tracking">
#
set(BEHAVIOR_PLUGIN_TARGET qrb_target_tracking)

add_library(${BEHAVIOR_PLUGIN_TARGET} SHARED
  src/qrb_target_tracking.cpp
  src/register_behaviors.cpp
)

target_compile_features(${BEHAVIOR_PLUGIN_TARGET}
  PUBLIC
    cxx_std_17
)

target_include_directories(${BEHAVIOR_PLUGIN_TARGET}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${BEHAVIOR_PLUGIN_TARGET}
  cv_bridge
  geometry_msgs
  moveit_pro_behavior_interface
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
)

target_link_libraries(${BEHAVIOR_PLUGIN_TARGET}
  ${OpenCV_LIBRARIES}
)

#
# Export the Behavior plugin description to the ament resource index.
#
pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface
  qrb_target_tracking_plugin_description.xml
)

#
# Follow-Me people controller executable
#
add_executable(followme_people_controller
  src/followme_people_controller.cpp
)

target_compile_features(followme_people_controller
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(followme_people_controller
  geometry_msgs
  moveit_studio_internal_msgs
  rclcpp
  std_msgs
)

#
# Install the Behavior plugin library.
#
install(
  TARGETS ${BEHAVIOR_PLUGIN_TARGET}
  EXPORT export_${BEHAVIOR_PLUGIN_TARGET}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#
# Install the controller executable.
#
install(
  TARGETS followme_people_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#
# Install public headers.
#
install(
  DIRECTORY include/
  DESTINATION include
)

#
# Install MoveIt Pro project resources.
#
install(
  DIRECTORY
    config
    description
    launch
    objectives
    waypoints
  DESTINATION share/${PROJECT_NAME}
)

#
# Install package-level files.
#
install(
  FILES
    qrb_target_tracking_plugin_description.xml
    README.md
  DESTINATION share/${PROJECT_NAME}
)

#
# Tests
#
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

#
# Export targets and dependencies for downstream packages.
#
ament_export_targets(
  export_${BEHAVIOR_PLUGIN_TARGET}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  behaviortree_cpp
  cv_bridge
  geometry_msgs
  moveit_pro_behavior_interface
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${BEHAVIOR_PLUGIN_TARGET})

ament_package()
