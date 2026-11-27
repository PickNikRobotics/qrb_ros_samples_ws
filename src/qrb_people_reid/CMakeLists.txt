cmake_minimum_required(VERSION 3.16)
project(qrb_people_reid)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(moveit_pro_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(qrb_ros_people_reid REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/qrb_people_reid.cpp
  src/register_behaviors.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge
  moveit_pro_behavior_interface
  pluginlib
  qrb_ros_people_reid
  rclcpp
  sensor_msgs
  vision_msgs
)

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface
  qrb_people_reid_plugin_description.xml
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES qrb_people_reid_plugin_description.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_dependencies(
  cv_bridge moveit_pro_behavior_interface pluginlib
  qrb_ros_people_reid rclcpp sensor_msgs vision_msgs
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
