#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

TEST(QrbPeopleReidPlugin, LoaderIsDiscoverable)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
    "moveit_pro_behavior_interface",
    "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");

  EXPECT_NO_THROW({
    auto instance = loader.createSharedInstance(
      "qrb_people_reid/QrbPeopleReidBehaviorsLoader");
    EXPECT_NE(instance, nullptr);
  });
}
