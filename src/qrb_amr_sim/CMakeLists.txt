cmake_minimum_required(VERSION 3.22)
project(qrb_amr_sim)

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

install(PROGRAMS scripts/twist_stamper.py DESTINATION lib/${PROJECT_NAME})

ament_package()
