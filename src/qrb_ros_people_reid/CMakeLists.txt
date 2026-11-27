cmake_minimum_required(VERSION 3.8)
project(qrb_ros_people_reid)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(qrb_ros_tensor_list_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExtractFeature.srv"
  "srv/ComputeSimilarity.srv"
  DEPENDENCIES sensor_msgs std_msgs
)

# Build the component library
add_library(people_reid_component SHARED src/people_reid_node.cpp)
target_include_directories(people_reid_component PRIVATE include)

ament_target_dependencies(people_reid_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  qrb_ros_tensor_list_msgs
  cv_bridge
)

target_link_libraries(people_reid_component ${OpenCV_LIBRARIES})

# Link generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(people_reid_component "${cpp_typesupport_target}")

# Register component
rclcpp_components_register_node(people_reid_component
  PLUGIN "qrb_ros::people_reid::PeopleReIDNode"
  EXECUTABLE people_reid_node_exe
)

# Build test client executable
add_executable(test_reid_client src/test_reid_client.cpp)
target_include_directories(test_reid_client PRIVATE include)

ament_target_dependencies(test_reid_client
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(test_reid_client ${OpenCV_LIBRARIES})

# Link generated service interfaces to test client
rosidl_get_typesupport_target(cpp_typesupport_target_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(test_reid_client "${cpp_typesupport_target_client}")

# Install targets
install(TARGETS
  people_reid_component
  test_reid_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  qrb_ros_tensor_list_msgs
  cv_bridge
  rosidl_default_runtime
)

ament_package()
