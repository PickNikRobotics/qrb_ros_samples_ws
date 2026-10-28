#pragma once

#include <map>
#include <string>

#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <tl_expected/expected.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace qrb_pick_place_core
{
/**
 * @brief Plans joint-space trajectories with MoveIt Pro Core's ProRRT planner.
 *
 * Replaces the planning half of MoveIt 2's MoveGroupInterface (setNamedTarget / setJointValueTarget + plan()).
 * Planning runs in-process against the robot model; no move_group node is involved.
 */
class MotionPlanner
{
public:
  /**
   * @param model Robot model. Acceleration bounds are applied to it from @p max_accelerations, because URDF has no
   *        acceleration limits and Core does not read joint_limits.yaml (move_group did that in MoveIt 2).
   * @param max_accelerations Per-joint symmetric acceleration limit (rad/s^2), keyed by joint name.
   */
  MotionPlanner(moveit_pro::base::RobotModelPtr model, const std::map<std::string, double>& max_accelerations,
                double velocity_scale, double acceleration_scale);

  /** @brief Plan @p group from @p start to the SRDF group_state @p state_name. */
  [[nodiscard]] tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
  planToNamedState(const moveit_pro::base::RobotState& start, const std::string& group,
                   const std::string& state_name) const;

  /** @brief Plan @p group from @p start to @p joint_angles_rad; joints not listed keep their start value. */
  [[nodiscard]] tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
  planToJointAngles(const moveit_pro::base::RobotState& start, const std::string& group,
                    const std::map<std::string, double>& joint_angles_rad) const;

private:
  tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
  plan(const moveit_pro::base::RobotState& start, const moveit_pro::base::JointModelGroup& group,
       const moveit_pro::base::RobotState& goal) const;

  moveit_pro::base::RobotModelPtr model_;
  double velocity_scale_;
  double acceleration_scale_;
};
}  // namespace qrb_pick_place_core
