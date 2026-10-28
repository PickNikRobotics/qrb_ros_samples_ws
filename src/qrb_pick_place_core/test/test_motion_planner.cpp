#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include "qrb_pick_place_core/motion_planner.hpp"

namespace
{
constexpr double kDeg = M_PI / 180.0;
const std::string kArm = "rm_group_controller";
const std::string kHand = "hand_controller";
// Qualcomm's config/joint_limits.yaml: max_acceleration 3.0 on every joint.
const std::map<std::string, double> kAccel{ { "joint1", 3.0 },  { "joint2", 3.0 },  { "joint3", 3.0 },
                                            { "joint4", 3.0 },  { "joint5", 3.0 },  { "joint6", 3.0 },
                                            { "joint01", 3.0 }, { "joint11", 3.0 }, { "joint02", 3.0 },
                                            { "joint22", 3.0 } };

std::string readFile(const std::string& path)
{
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

class MotionPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string fixtures = std::string(TEST_FIXTURES_DIR);
    const auto urdf = urdf::parseURDF(readFile(fixtures + "/rml63.urdf"));
    ASSERT_NE(urdf, nullptr);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, readFile(fixtures + "/rml_63.srdf")));
    model_ = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
    start_ = std::make_unique<moveit_pro::base::RobotState>(model_);
    start_->setToDefaultValues();
    start_->update();
  }

  const trajectory_msgs::msg::JointTrajectoryPoint& last(const trajectory_msgs::msg::JointTrajectory& t)
  {
    return t.points.back();
  }

  static double position(const trajectory_msgs::msg::JointTrajectory& t,
                         const trajectory_msgs::msg::JointTrajectoryPoint& p, const std::string& joint)
  {
    for (size_t i = 0; i < t.joint_names.size(); ++i)
    {
      if (t.joint_names[i] == joint)
      {
        return p.positions[i];
      }
    }
    ADD_FAILURE() << "joint " << joint << " missing from trajectory";
    return NAN;
  }

  static double duration(const trajectory_msgs::msg::JointTrajectory& t)
  {
    const auto& d = t.points.back().time_from_start;
    return d.sec + d.nanosec * 1e-9;
  }

  moveit_pro::base::RobotModelPtr model_;
  std::unique_ptr<moveit_pro::base::RobotState> start_;
};

TEST_F(MotionPlannerTest, PlansToReadyFromDefault)
{
  const qrb_pick_place_core::MotionPlanner planner(model_, kAccel, 0.3, 0.3);
  const auto trajectory = planner.planToNamedState(*start_, kArm, "ready");
  ASSERT_TRUE(trajectory.has_value()) << trajectory.error();
  // Values from rml_63.srdf group_state "ready".
  const std::map<std::string, double> ready{ { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", -2.5 },
                                             { "joint4", 0.0 }, { "joint5", 1.0 }, { "joint6", 0.0 } };
  for (const auto& [joint, value] : ready)
  {
    EXPECT_NEAR(position(*trajectory, last(*trajectory), joint), value, 1e-6) << joint;
    EXPECT_NEAR(position(*trajectory, trajectory->points.front(), joint), start_->getVariablePosition(joint), 1e-6)
        << joint;
  }
}

TEST_F(MotionPlannerTest, UnknownNamedStateFails)
{
  const qrb_pick_place_core::MotionPlanner planner(model_, kAccel, 0.3, 0.3);
  const auto trajectory = planner.planToNamedState(*start_, kArm, "nope");
  ASSERT_FALSE(trajectory.has_value());
  EXPECT_NE(trajectory.error().find("nope"), std::string::npos) << trajectory.error();
}

TEST_F(MotionPlannerTest, PlansToQualcommPickAngles)
{
  const qrb_pick_place_core::MotionPlanner planner(model_, kAccel, 0.3, 0.3);
  // qrb_ros_arm_pick_place.cpp getPickJointAngles().
  const std::map<std::string, double> pick{ { "joint1", 0.0 },          { "joint2", -46.0 * kDeg },
                                            { "joint3", -114.0 * kDeg }, { "joint4", 0.0 },
                                            { "joint5", 74.0 * kDeg },   { "joint6", 0.0 } };
  const auto trajectory = planner.planToJointAngles(*start_, kArm, pick);
  ASSERT_TRUE(trajectory.has_value()) << trajectory.error();
  for (const auto& [joint, value] : pick)
  {
    EXPECT_NEAR(position(*trajectory, last(*trajectory), joint), value, 1e-6) << joint;
  }
}

TEST_F(MotionPlannerTest, GripperStatesDriveMimicFingers)
{
  // The node sends only joint01 to the gripper controller; the SRDF states and URDF mimic joints must still put
  // the other fingers where Qualcomm's original independent values had them.
  moveit_pro::base::RobotState open(*start_);
  ASSERT_TRUE(open.setToDefaultValues(model_->getJointModelGroup(kHand), "open"));
  open.update();
  EXPECT_NEAR(open.getVariablePosition("joint01"), 0.0, 1e-6);
  EXPECT_NEAR(open.getVariablePosition("joint11"), -0.25, 1e-6);
  EXPECT_NEAR(open.getVariablePosition("joint02"), 0.0, 1e-6);
  EXPECT_NEAR(open.getVariablePosition("joint22"), 0.25, 1e-6);
  moveit_pro::base::RobotState close(*start_);
  ASSERT_TRUE(close.setToDefaultValues(model_->getJointModelGroup(kHand), "close"));
  close.update();
  EXPECT_NEAR(close.getVariablePosition("joint01"), 0.2, 1e-6);
  EXPECT_NEAR(close.getVariablePosition("joint11"), 0.0, 1e-6);
  EXPECT_NEAR(close.getVariablePosition("joint02"), -0.2, 1e-6);
  EXPECT_NEAR(close.getVariablePosition("joint22"), 0.0, 1e-6);
}

TEST_F(MotionPlannerTest, RespectsScaling)
{
  const auto slow = qrb_pick_place_core::MotionPlanner(model_, kAccel, 0.3, 0.3).planToNamedState(*start_, kArm, "ready");
  const auto fast = qrb_pick_place_core::MotionPlanner(model_, kAccel, 1.0, 1.0).planToNamedState(*start_, kArm, "ready");
  ASSERT_TRUE(slow.has_value()) << slow.error();
  ASSERT_TRUE(fast.has_value()) << fast.error();
  EXPECT_GT(duration(*slow), duration(*fast) * 1.5) << "slow " << duration(*slow) << " fast " << duration(*fast);
}

TEST_F(MotionPlannerTest, UnknownJointFails)
{
  const qrb_pick_place_core::MotionPlanner planner(model_, kAccel, 0.3, 0.3);
  const auto trajectory = planner.planToJointAngles(*start_, kArm, { { "joint9", 0.0 } });
  ASSERT_FALSE(trajectory.has_value());
  EXPECT_NE(trajectory.error().find("joint9"), std::string::npos) << trajectory.error();
}

}  // namespace
