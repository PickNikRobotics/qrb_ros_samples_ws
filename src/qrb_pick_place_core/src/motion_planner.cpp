#include "qrb_pick_place_core/motion_planner.hpp"

#include <stdexcept>
#include <utility>

#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <pro_rrt/pro_rrt.hpp>

namespace qrb_pick_place_core
{
MotionPlanner::MotionPlanner(moveit_pro::base::RobotModelPtr model,
                             const std::map<std::string, double>& max_accelerations, double velocity_scale,
                             double acceleration_scale)
  : model_(std::move(model)), velocity_scale_(velocity_scale), acceleration_scale_(acceleration_scale)
{
  for (const auto& [joint, max_acceleration] : max_accelerations)
  {
    if (!model_->hasJointModel(joint))
    {
      throw std::invalid_argument("acceleration limit given for unknown joint '" + joint + "'");
    }
    auto bounds = model_->getVariableBounds(joint);
    bounds.acceleration_bounded = true;
    bounds.min_acceleration = -max_acceleration;
    bounds.max_acceleration = max_acceleration;
    model_->getJointModel(joint)->setVariableBounds(joint, bounds);
  }
}

tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
MotionPlanner::planToNamedState(const moveit_pro::base::RobotState& start, const std::string& group,
                                const std::string& state_name) const
{
  const auto* jmg = model_->getJointModelGroup(group);
  if (jmg == nullptr)
  {
    return tl::make_unexpected("Unknown planning group '" + group + "'.");
  }
  moveit_pro::base::RobotState goal(start);
  if (!goal.setToDefaultValues(jmg, state_name))
  {
    return tl::make_unexpected("Group '" + group + "' has no SRDF group_state named '" + state_name + "'.");
  }
  goal.update();
  return plan(start, *jmg, goal);
}

tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
MotionPlanner::planToJointAngles(const moveit_pro::base::RobotState& start, const std::string& group,
                                 const std::map<std::string, double>& joint_angles_rad) const
{
  const auto* jmg = model_->getJointModelGroup(group);
  if (jmg == nullptr)
  {
    return tl::make_unexpected("Unknown planning group '" + group + "'.");
  }
  for (const auto& [joint, value] : joint_angles_rad)
  {
    if (!jmg->hasJointModel(joint))
    {
      return tl::make_unexpected("Joint '" + joint + "' is not in planning group '" + group + "'.");
    }
  }
  moveit_pro::base::RobotState goal(start);
  goal.setVariablePositions(joint_angles_rad);
  goal.update();
  return plan(start, *jmg, goal);
}

tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
MotionPlanner::plan(const moveit_pro::base::RobotState& start, const moveit_pro::base::JointModelGroup& group,
                    const moveit_pro::base::RobotState& goal) const
{
  // Empty scene: like Qualcomm's original, the sample plans against robot self-collision only.
  moveit_pro::base::planning_scene::PlanningScene scene(model_);
  scene.getCurrentStateNonConst() = start;

  pro_rrt::TrajectoryParams trajectory_params;
  trajectory_params.velocity_scale_factor = velocity_scale_;
  trajectory_params.acceleration_scale_factor = acceleration_scale_;

  auto trajectory =
      pro_rrt::planTrajectoryToJointGoal(group, start, goal, pro_rrt::RRTParams{}, {}, trajectory_params, scene);
  if (!trajectory)
  {
    return tl::make_unexpected(trajectory.error().message);
  }
  return std::move(*trajectory);
}
}  // namespace qrb_pick_place_core
