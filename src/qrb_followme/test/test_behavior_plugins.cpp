#include <gtest/gtest.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>

TEST(QrbTargetTrackingPlugin, LoaderIsDiscoverable)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
    "moveit_pro_behavior_interface",
    "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  EXPECT_NO_THROW({
    auto instance = loader.createSharedInstance(
      "qrb_target_tracking/QrbTargetTrackingBehaviorsLoader");
    EXPECT_NE(instance, nullptr);
  });
}
