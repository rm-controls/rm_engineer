#include "xr_controller_ros/xr_controller.h"

#include <Eigen/Geometry>
#include <moveit/kinematics_base/kinematics_base.h>

#include <cmath>
#include <limits>

namespace
{
constexpr char kLeftToolLink[] = "left_tool0_link";
constexpr char kRightToolLink[] = "right_tool0_link";

geometry_msgs::Pose PoseFromTransform(const Eigen::Isometry3d& transform)
{
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

double JointDistance(const std::vector<double>& first, const std::vector<double>& second)
{
  if (first.size() != second.size())
  {
    return std::numeric_limits<double>::infinity();
  }
  double squared_distance = 0.0;
  for (size_t index = 0; index < first.size(); ++index)
  {
    const double difference = first[index] - second[index];
    squared_distance += difference * difference;
  }
  return std::sqrt(squared_distance);
}

}  // namespace

namespace xr_controller_ros
{
XrController::XrController(ros::NodeHandle& nh) : robot_model_loader_("robot_description")
{
  left_control_cmd_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/controllers/left_arm_controller/command", 1);
  right_control_cmd_pub_ =
      nh.advertise<trajectory_msgs::JointTrajectory>("/controllers/right_arm_controller/command", 1);
  // rm_manual and the robot-state publisher use the global topic. Do not
  // resolve this against rm_manual's private namespace.
  joint_state_sub_ = nh.subscribe("/joint_states", 1, &XrController::jointStateCallback, this);
  nh.param("xr_position_scale_x", position_scale_x_, 1.0);
  nh.param("xr_position_scale_y", position_scale_y_, 1.0);
  nh.param("xr_position_scale_z", position_scale_z_, 1.0);
  nh.param("xr_trajectory_duration", trajectory_duration_, 0.1);
  nh.param("xr_max_joint_step", max_joint_step_, 0.3);

  robot_model_ = robot_model_loader_.getModel();
  if (!robot_model_)
  {
    ROS_ERROR("XR controller: failed to load robot_description");
    return;
  }
  left_arm_group_ = robot_model_->getJointModelGroup("left_arm");
  right_arm_group_ = robot_model_->getJointModelGroup("right_arm");
  if (!left_arm_group_ || !right_arm_group_)
  {
    ROS_ERROR("XR controller: MoveIt groups left_arm/right_arm are unavailable");
    return;
  }
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
}

void XrController::checkOnOff()
{
  const bool mark = control_data_.right_gesture_middle_pinch && control_data_.left_gesture_middle_pinch;
  if (mark && !last_mark_)
  {
    mark_timing_ = ros::Time::now();
    mark_triggered_ = false;
  }
  if (mark && !mark_triggered_ && (ros::Time::now() - mark_timing_).toSec() >= 1.0)
  {
    is_start_ = !is_start_;
    mark_triggered_ = true;
  }
  last_mark_ = mark;
}

void XrController::jointStateCallback(const sensor_msgs::JointStateConstPtr& data)
{
  joint_state_ = *data;
  has_joint_state_ = !joint_state_.name.empty() && joint_state_.name.size() == joint_state_.position.size();
}

bool XrController::setRobotStateFromJointState()
{
  if (!has_joint_state_ || !robot_state_)
  {
    return false;
  }
  robot_state_->setVariablePositions(joint_state_.name, joint_state_.position);
  robot_state_->update();
  return true;
}

bool XrController::captureReference()
{
  if (!setRobotStateFromJointState())
  {
    return false;
  }
  left_hand_reference_ = control_data_.left_position;
  right_hand_reference_ = control_data_.right_position;
  left_ee_reference_ = PoseFromTransform(robot_state_->getGlobalLinkTransform(kLeftToolLink));
  right_ee_reference_ = PoseFromTransform(robot_state_->getGlobalLinkTransform(kRightToolLink));
  has_last_left_solution_ = false;
  has_last_right_solution_ = false;
  has_reference_ = true;
  ROS_INFO("XR controller: reference captured");
  return true;
}

bool XrController::solveIkAndPublish(const geometry_msgs::Point& hand_position,
                                     const geometry_msgs::Quaternion& hand_quaternion,
                                     const geometry_msgs::Point& hand_reference,
                                     const geometry_msgs::Pose& ee_reference, const robot_model::JointModelGroup* group,
                                     const std::string& tip_link, ros::Publisher& publisher)
{
  if (!setRobotStateFromJointState())
  {
    return false;
  }

  geometry_msgs::Pose target = ee_reference;
  target.position.x += position_scale_x_ * (hand_position.x - hand_reference.x);
  target.position.y += position_scale_y_ * (hand_position.y - hand_reference.y);
  target.position.z += position_scale_z_ * (hand_position.z - hand_reference.z);
  // target.orientation = hand_quaternion;
  target.orientation = ee_reference.orientation;
  const double norm_squared = target.orientation.x * target.orientation.x +
                              target.orientation.y * target.orientation.y +
                              target.orientation.z * target.orientation.z + target.orientation.w * target.orientation.w;
  if (norm_squared < 1e-8)
  {
    ROS_WARN_THROTTLE(1.0, "XR controller: received zero quaternion");
    return false;
  }

  const bool is_left_arm = tip_link == kLeftToolLink;
  std::vector<double>* last_solution = is_left_arm ? &last_left_solution_ : &last_right_solution_;
  bool* has_last_solution = is_left_arm ? &has_last_left_solution_ : &has_last_right_solution_;
  std::vector<double> seed;
  if (*has_last_solution)
  {
    seed = *last_solution;
  }
  else
  {
    robot_state_->copyJointGroupPositions(group, seed);
  }

  // KinematicsBase::getPositionIK is specified to return the solution nearest
  // to seed. The seed is the prior successful command, so the IK branch remains
  // continuous across 30 Hz XR samples.
  const kinematics::KinematicsBaseConstPtr solver = group->getSolverInstance();
  if (!solver)
  {
    ROS_ERROR_THROTTLE(1.0, "XR controller: no IK solver for %s", tip_link.c_str());
    return false;
  }
  const Eigen::Isometry3d global_target =
      Eigen::Translation3d(target.position.x, target.position.y, target.position.z) *
      Eigen::Quaterniond(target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z)
          .normalized();
  const Eigen::Isometry3d solver_target =
      robot_state_->getGlobalLinkTransform(solver->getBaseFrame()).inverse() * global_target;
  moveit_msgs::MoveItErrorCodes error_code;
  std::vector<double> solution;
  if (!solver->getPositionIK(PoseFromTransform(solver_target), seed, solution, error_code))
  {
    ROS_WARN_THROTTLE(1.0, "XR controller: no IK solution for %s", tip_link.c_str());
    return false;
  }
  if (JointDistance(solution, seed) > max_joint_step_)
  {
    ROS_WARN_THROTTLE(1.0, "XR controller: rejected discontinuous IK solution for %s", tip_link.c_str());
    return false;
  }

  robot_state_->setJointGroupPositions(group, solution);
  if (!robot_state_->satisfiesBounds(group))
  {
    ROS_WARN_THROTTLE(1.0, "XR controller: IK result violates joint limits");
    return false;
  }
  *last_solution = solution;
  *has_last_solution = true;

  trajectory_msgs::JointTrajectory trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.joint_names = group->getVariableNames();
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = solution;
  point.time_from_start = ros::Duration(trajectory_duration_);
  trajectory.points.push_back(point);
  publisher.publish(trajectory);
  return true;
}

bool XrController::update(const rm_msgs::XrControllerData& data)
{
  control_data_ = data;
  const bool was_started = is_start_;
  checkOnOff();
  if (!is_start_)
  {
    has_reference_ = false;
    return false;
  }
  if (!data.is_valid || !data.is_left_valid || !data.is_right_valid)
  {
    ROS_WARN_THROTTLE(1.0, "XR controller: invalid hand tracking");
    return false;
  }
  if (!robot_state_ || !has_joint_state_)
  {
    ROS_WARN_THROTTLE(1.0, "XR controller: waiting for joint_states");
    return false;
  }
  if ((!was_started || !has_reference_) && !captureReference())
  {
    return false;
  }

  const bool left_ok = solveIkAndPublish(data.left_position, data.left_quaternion, left_hand_reference_,
                                         left_ee_reference_, left_arm_group_, kLeftToolLink, left_control_cmd_pub_);
  const bool right_ok =
      solveIkAndPublish(data.right_position, data.right_quaternion, right_hand_reference_, right_ee_reference_,
                        right_arm_group_, kRightToolLink, right_control_cmd_pub_);
  return left_ok && right_ok;
}

}  // namespace xr_controller_ros
