// Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
//
// Ported from MoveIt 2 (MoveGroupInterface + move_group) to MoveIt Pro Core. Step sequence, joint targets, and log
// text are unchanged from Qualcomm's qrb_ros_samples/robotics/simulation_sample_pick_and_place.

#include <chrono>
#include <cmath>  // For M_PI
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_pro/planning_scene_monitor/current_state_monitor.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <srdfdom/model.h>
#include <tf2_ros/buffer.h>
#include <urdf_parser/urdf_parser.h>

#include "qrb_pick_place_core/motion_planner.hpp"

namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
constexpr double kStateTimeoutS = 10.0;
constexpr auto kResultTimeout = std::chrono::seconds(60);
constexpr const char* kArmAction = "/rm_group_controller/follow_joint_trajectory";
constexpr const char* kGripperAction = "/hand_controller/follow_joint_trajectory";

/** @brief Build the robot model from the robot_description / robot_description_semantic parameters. */
moveit_pro::base::RobotModelPtr loadRobotModel(const rclcpp::Node::SharedPtr& node)
{
  const auto urdf_string = node->get_parameter("robot_description").as_string();
  const auto srdf_string = node->get_parameter("robot_description_semantic").as_string();
  const auto urdf = urdf::parseURDF(urdf_string);
  if (!urdf)
  {
    throw std::runtime_error("Failed to parse the robot_description parameter as URDF.");
  }
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, srdf_string))
  {
    throw std::runtime_error("Failed to parse the robot_description_semantic parameter as SRDF.");
  }
  return std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
}

/** @brief Read joint_limits.yaml-style max_acceleration parameters (joint_limits.<joint>.max_acceleration). */
std::map<std::string, double> loadAccelerationLimits(const rclcpp::Node::SharedPtr& node,
                                                     const moveit_pro::base::RobotModel& model)
{
  std::map<std::string, double> limits;
  for (const auto& joint : model.getActiveJointModelNames())
  {
    const auto name = "joint_limits." + joint + ".max_acceleration";
    if (node->has_parameter(name))
    {
      limits[joint] = node->get_parameter(name).as_double();
    }
  }
  return limits;
}
}  // namespace

/**
 * @brief Robot control class that encapsulates pick and place operations
 */
class RobotController
{
public:
  ~RobotController()
  {
    state_executor_.cancel();
    if (state_thread_.joinable()) {
      state_thread_.join();
    }
  }

  RobotController(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , logger_(rclcpp::get_logger("qrb_ros_arm_pick_place"))
    , model_(loadRobotModel(node))
    , state_node_(std::make_shared<rclcpp::Node>("qrb_ros_arm_pick_place_state",
          rclcpp::NodeOptions().parameter_overrides({ { "use_sim_time",
              node->get_parameter("use_sim_time").as_bool() } })))
    , state_monitor_(std::make_shared<moveit_pro::planning_scene_monitor::CurrentStateMonitor>(
          state_node_, model_, std::make_shared<tf2_ros::Buffer>(node->get_clock()),
          node->get_parameter("use_sim_time").as_bool()))
    , planner_(model_, loadAccelerationLimits(node, *model_), kVelocityScaling, kAccelerationScaling)
    , arm_client_(rclcpp_action::create_client<FollowJointTrajectory>(node, kArmAction))
    , gripper_client_(rclcpp_action::create_client<FollowJointTrajectory>(node, kGripperAction))
  {
    setupMoveGroups();
  }

  /**
   * @brief Execute the complete pick and place task
   */
  bool executePickAndPlace()
  {
    try {
      RCLCPP_INFO(logger_, "Starting pick and place task...");

      // Step 1: Move to ready state
      if (!moveToNamedTarget(kArmGroup, "ready", "Ready State")) {
        return false;
      }

      // Step 2: Open gripper
      if (!moveToNamedTarget(kGripperGroup, "open", "Open Gripper")) {
        return false;
      }

      // Step 3: Move to target joint angles (pick position)
      if (!moveToJointAngles(getPickJointAngles(), "Pick Position")) {
        return false;
      }

      // Step 4: Close gripper (grasp)
      if (!moveToNamedTarget(kGripperGroup, "close", "Close Gripper")) {
        return false;
      }

      // Step 5: Lift object
      if (!moveToJointAngles(getRaiseJointAngles(), "Lift Object")) {
        return false;
      }

      // Step 6: Move to place position
      if (!moveToJointAngles(getPlaceJointAngles(), "Place Position")) {
        return false;
      }

      // Step 7: Open gripper (release)
      if (!moveToNamedTarget(kGripperGroup, "open", "Release Object")) {
        return false;
      }

      // Step 8: Lift again
      if (!moveToJointAngles(getReRaiseJointAngles(), "Lift Again")) {
        return false;
      }

      // Step 9: Return to ready state
      if (!moveToNamedTarget(kArmGroup, "ready", "Return to Ready State")) {
        return false;
      }

      RCLCPP_INFO(logger_, "Pick and place task completed!");
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Error during execution: %s", e.what());
      return false;
    }
  }

private:
  // Planning group names are the SRDF groups; each is driven by the controller of the same name.
  static constexpr const char * kArmGroup = "rm_group_controller";
  static constexpr const char * kGripperGroup = "hand_controller";
  static constexpr double kVelocityScaling = 0.3;
  static constexpr double kAccelerationScaling = 0.3;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Logger logger_;
  moveit_pro::base::RobotModelPtr model_;
  // MoveGroupInterface spun its own executor to receive /joint_states; here the state monitor gets a dedicated node
  // spun in the background, so the main node stays free for spin_until_future_complete on action calls.
  rclcpp::Node::SharedPtr state_node_;
  rclcpp::executors::SingleThreadedExecutor state_executor_;
  std::thread state_thread_;
  std::shared_ptr<moveit_pro::planning_scene_monitor::CurrentStateMonitor> state_monitor_;
  qrb_pick_place_core::MotionPlanner planner_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;

  /**
   * @brief Start state monitoring and wait for the controllers
   */
  void setupMoveGroups()
  {
    state_executor_.add_node(state_node_);
    state_thread_ = std::thread([this] { state_executor_.spin(); });
    state_monitor_->startStateMonitor("/joint_states");
    if (!state_monitor_->waitForCompleteState(kStateTimeoutS)) {
      throw std::runtime_error("No complete robot state on /joint_states within 10 s. Is the simulator running?");
    }
    for (const auto & [client, name] : { std::pair{ arm_client_, kArmAction }, std::pair{ gripper_client_, kGripperAction } }) {
      if (!client->wait_for_action_server(std::chrono::seconds(10))) {
        throw std::runtime_error(std::string("Action server ") + name + " is not available.");
      }
    }

    RCLCPP_INFO(logger_, "Planning pipeline: MoveIt Pro Core");
    RCLCPP_INFO(logger_, "Planner ID: ProRRT");
    RCLCPP_INFO(logger_, "Velocity / acceleration scaling: %.2f / %.2f", kVelocityScaling, kAccelerationScaling);
  }

  /**
   * @brief Move to a named target
   */
  bool moveToNamedTarget(const std::string & group, const std::string & target_name,
      const std::string & description)
  {
    RCLCPP_INFO(logger_, "Step: %s", description.c_str());

    const auto trajectory = planner_.planToNamedState(*state_monitor_->getCurrentState(), group, target_name);
    return finishStep(group, trajectory, description);
  }

  /**
   * @brief Move to specified joint angles
   */
  bool moveToJointAngles(const std::map<std::string, double> & joint_angles_deg,
      const std::string & description)
  {
    RCLCPP_INFO(logger_, "Step: %s", description.c_str());

    // Convert degrees to radians
    std::map<std::string, double> joint_angles_rad = convertToRadians(joint_angles_deg);

    // Print target angles
    for (const auto & [joint_name, angle_deg] : joint_angles_deg) {
      RCLCPP_INFO(logger_, "Target %s: %.2f° (%.4f rad)", joint_name.c_str(), angle_deg,
          joint_angles_rad[joint_name]);
    }

    const auto trajectory =
      planner_.planToJointAngles(*state_monitor_->getCurrentState(), kArmGroup, joint_angles_rad);
    return finishStep(kArmGroup, trajectory, description);
  }

  bool finishStep(const std::string & group,
      const tl::expected<trajectory_msgs::msg::JointTrajectory, std::string> & trajectory,
      const std::string & description)
  {
    if (!trajectory) {
      RCLCPP_ERROR(logger_, "%s failed: planning error: %s", description.c_str(), trajectory.error().c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "Planning successful! Executing...");
    const auto error = execute(group, *trajectory);
    if (!error.empty()) {
      RCLCPP_ERROR(logger_, "%s failed: %s", description.c_str(), error.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "%s completed", description.c_str());
    return true;
  }

  /**
   * @brief Execute a trajectory on the group's controller; returns an empty string on success.
   */
  std::string execute(const std::string & group, const trajectory_msgs::msg::JointTrajectory & trajectory)
  {
    const bool is_arm = group == kArmGroup;
    const auto & client = is_arm ? arm_client_ : gripper_client_;
    const std::string action = is_arm ? kArmAction : kGripperAction;
    FollowJointTrajectory::Goal goal;
    goal.trajectory = trajectory;

    auto goal_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node_, goal_future, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return "no response from " + action;
    }
    const auto handle = goal_future.get();
    if (!handle) {
      return action + " rejected the trajectory";
    }
    auto result_future = client->async_get_result(handle);
    if (rclcpp::spin_until_future_complete(node_, result_future, kResultTimeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return "no result from " + action + " within 60 s";
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || wrapped.result->error_code != 0) {
      return action + " failed with error code " +
             std::to_string(wrapped.result->error_code) + ": " + wrapped.result->error_string;
    }
    return "";
  }

  /**
   * @brief Convert angles from degrees to radians
   */
  std::map<std::string, double> convertToRadians(const std::map<std::string, double> & angles_deg)
  {
    std::map<std::string, double> angles_rad;
    for (const auto & [joint_name, angle_deg] : angles_deg) {
      angles_rad[joint_name] = angle_deg * M_PI / 180.0;
    }
    return angles_rad;
  }

  /**
   * @brief Get joint angles for pick position
   */
  std::map<std::string, double> getPickJointAngles()
  {
    return {
      { "joint1", 0.0 },     // 0°
      { "joint2", -46.0 },   // -46°
      { "joint3", -114.0 },  // -114°
      { "joint4", 0.0 },     // 0°
      { "joint5", 74.0 },    // 74°
      { "joint6", 0.0 }      // 0°
    };
  }

  /**
   * @brief Get joint angles for lift position
   */
  std::map<std::string, double> getRaiseJointAngles()
  {
    return {
      { "joint1", -22.0 },  // -22°
      { "joint2", -37.0 },  // -37°
      { "joint3", -48.0 },  // -48°
      { "joint4", 0.0 },    // 0°
      { "joint5", 0.0 },    // 0°
      { "joint6", 0.0 }     // 0°
    };
  }

  /**
   * @brief Get joint angles for place position
   */
  std::map<std::string, double> getPlaceJointAngles()
  {
    return {
      { "joint1", -31.0 },  // -31°
      { "joint2", -42.0 },  // -42°
      { "joint3", -59.0 },  // -59°
      { "joint4", 0.0 },    // 0°
      { "joint5", 0.0 },    // 0°
      { "joint6", 0.0 }     // 0°
    };
  }

  /**
   * @brief Get joint angles for re-lift position
   */
  std::map<std::string, double> getReRaiseJointAngles()
  {
    return {
      { "joint1", -22.0 },  // -22°
      { "joint2", -35.0 },  // -35°
      { "joint3", -52.0 },  // -52°
      { "joint4", 0.0 },    // 0°
      { "joint5", 0.0 },    // 0°
      { "joint6", 0.0 }     // 0°
    };
  }
};

/**
 * @brief Main function
 */
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<rclcpp::Node>("qrb_ros_arm_pick_place",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create robot controller and execute task
  bool success = false;
  try {
    RobotController controller(node);
    success = controller.executePickAndPlace();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
  }

  // Shutdown ROS 2
  rclcpp::shutdown();

  return success ? 0 : 1;
}
