#include <qrb_target_tracking/qrb_target_tracking.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace qrb_target_tracking
{
class QrbTargetTrackingBehaviorsLoader
  : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
    BT::BehaviorTreeFactory & factory,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext> & shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<QrbTargetTracking>(
      factory, "QrbTargetTracking", shared_resources);
  }
};
}  // namespace qrb_target_tracking

PLUGINLIB_EXPORT_CLASS(
  qrb_target_tracking::QrbTargetTrackingBehaviorsLoader,
  moveit_pro::behaviors::SharedResourcesNodeLoaderBase)
