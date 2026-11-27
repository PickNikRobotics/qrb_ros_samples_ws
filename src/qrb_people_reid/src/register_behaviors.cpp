#include <qrb_people_reid/qrb_people_reid.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace qrb_people_reid
{
class QrbPeopleReidBehaviorsLoader
  : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
    BT::BehaviorTreeFactory& factory,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<QrbPeopleReid>(
      factory, "QrbPeopleReid", shared_resources);
  }
};
}  // namespace qrb_people_reid

PLUGINLIB_EXPORT_CLASS(
  qrb_people_reid::QrbPeopleReidBehaviorsLoader,
  moveit_pro::behaviors::SharedResourcesNodeLoaderBase)
