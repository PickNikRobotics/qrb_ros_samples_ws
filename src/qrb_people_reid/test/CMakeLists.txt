ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
if(TARGET test_behavior_plugins)
  target_compile_features(test_behavior_plugins PUBLIC cxx_std_17)
  target_link_libraries(test_behavior_plugins ${PROJECT_NAME})
  ament_target_dependencies(test_behavior_plugins
    moveit_pro_behavior_interface
    pluginlib
  )
endif()
