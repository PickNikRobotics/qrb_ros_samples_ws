#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <qrb_yolo_behavior/qrb_yolo_behavior.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace qrb_yolo_behavior
{
class QrbYoloBehaviorBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<QrbYoloBehavior>(factory, "QrbYoloBehavior", shared_resources);
    
  }
};
}  // namespace qrb_yolo_behavior

PLUGINLIB_EXPORT_CLASS(qrb_yolo_behavior::QrbYoloBehaviorBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
