/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

#include "engineer_middleware/motion.h"
#include "engineer_middleware/planning_scene.h"
#include <string>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

namespace engineer_middleware
{
class Step
{
public:
  Step(const XmlRpc::XmlRpcValue& step, const XmlRpc::XmlRpcValue& scenes, tf2_ros::Buffer& tf,
       moveit::planning_interface::MoveGroupInterface& arm_group, ChassisInterface& chassis_interface,
       ros::Publisher& hand_pub, ros::Publisher& end_effector_pub, ros::Publisher& gimbal_pub, ros::Publisher& gpio_pub,
       ros::Publisher& reversal_pub, ros::Publisher& stone_num_pub, ros::Publisher& planning_result_pub,
       ros::Publisher& point_cloud_pub)
    : planning_result_pub_(planning_result_pub), point_cloud_pub_(point_cloud_pub), arm_group_(arm_group)
  {
    ROS_ASSERT(step.hasMember("step"));
    step_name_ = static_cast<std::string>(step["step"]);
    if (step.hasMember("arm"))
    {
      if (step["arm"].hasMember("joints"))
        arm_motion_ = new JointMotion(step["arm"], arm_group);
      else if (step["arm"].hasMember("spacial_shape"))
        arm_motion_ = new SpaceEeMotion(step["arm"], arm_group, tf);
      else
        arm_motion_ = new EndEffectorMotion(step["arm"], arm_group, tf);
    }
    if (step.hasMember("chassis"))
      chassis_motion_ = new ChassisMotion(step["chassis"], chassis_interface);
    if (step.hasMember("hand"))
      hand_motion_ = new HandMotion(step["hand"], hand_pub);
    if (step.hasMember("end_effector"))
      end_effector_motion_ = new JointPositionMotion(step["end_effector"], end_effector_pub, tf);
    if (step.hasMember("stone_num"))
      stone_num_motion_ = new StoneNumMotion(step["stone_num"], stone_num_pub);
    if (step.hasMember("gimbal"))
      gimbal_motion_ = new GimbalMotion(step["gimbal"], gimbal_pub);
    if (step.hasMember("gripper"))
      gpio_motion_ = new GpioMotion(step["gripper"], gpio_pub);
    if (step.hasMember("reversal"))
      reversal_motion_ = new ReversalMotion(step["reversal"], reversal_pub);
    if (step.hasMember("scene_name"))
    {
      for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = scenes.begin(); it != scenes.end(); ++it)
        if (step["scene_name"] == it->first)
          planning_scene_ = new PlanningScene(it->second, arm_group);
    }
  }
  bool move()
  {
    bool success = true;
    if (arm_motion_)
    {
      success &= arm_motion_->move();
      if (!arm_motion_->getPointCloud2().data.empty())
      {
        sensor_msgs::PointCloud2 point_cloud2 = arm_motion_->getPointCloud2();
        point_cloud_pub_.publish(point_cloud2);
      }
      std_msgs::Int32 msg = arm_motion_->getPlanningResult();
      planning_result_pub_.publish(msg);
    }
    if (hand_motion_)
      success &= hand_motion_->move();
    if (end_effector_motion_)
      success &= end_effector_motion_->move();
    if (stone_num_motion_)
      success &= stone_num_motion_->move();
    if (chassis_motion_)
      success &= chassis_motion_->move();
    if (gimbal_motion_)
      success &= gimbal_motion_->move();
    if (gpio_motion_)
      success &= gpio_motion_->move();
    if (reversal_motion_)
      success &= reversal_motion_->move();
    if (planning_scene_)
      planning_scene_->add();
    return success;
  }
  void stop()
  {
    if (arm_motion_)
      arm_motion_->stop();
    if (hand_motion_)
      hand_motion_->stop();
    if (chassis_motion_)
      chassis_motion_->stop();
  }

  void deleteScene()
  {
    std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects =
        planning_scene_interface_.getAttachedObjects();
    for (auto iter = attached_objects.begin(); iter != attached_objects.end(); ++iter)
    {
      arm_group_.detachObject(iter->first);
    }
    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());
  }
  bool isFinish()
  {
    bool success = true;
    if (arm_motion_)
      success &= arm_motion_->isFinish();
    if (hand_motion_)
      success &= hand_motion_->isFinish();
    if (end_effector_motion_)
      success &= end_effector_motion_->isFinish();
    if (chassis_motion_)
      success &= chassis_motion_->isFinish();
    if (gimbal_motion_)
      success &= gimbal_motion_->isFinish();
    if (reversal_motion_)
      success &= reversal_motion_->isFinish();
    return success;
  }
  bool checkTimeout(ros::Duration period)
  {
    bool success = true;
    if (arm_motion_)
      success &= arm_motion_->checkTimeout(period);
    if (hand_motion_)
      success &= hand_motion_->checkTimeout(period);
    if (end_effector_motion_)
      success &= end_effector_motion_->checkTimeout(period);
    if (chassis_motion_)
      success &= chassis_motion_->checkTimeout(period);
    if (gimbal_motion_)
      success &= gimbal_motion_->checkTimeout(period);
    return success;
  }

  std::string getName()
  {
    return step_name_;
  }

private:
  std::string step_name_;
  ros::Publisher planning_result_pub_;
  ros::Publisher point_cloud_pub_;
  MoveitMotionBase* arm_motion_{};
  HandMotion* hand_motion_{};
  JointPositionMotion* end_effector_motion_{};
  StoneNumMotion* stone_num_motion_{};
  ChassisMotion* chassis_motion_{};
  GimbalMotion* gimbal_motion_{};
  GpioMotion* gpio_motion_{};
  ReversalMotion* reversal_motion_{};
  PlanningScene* planning_scene_{};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface& arm_group_;
};

}  // namespace engineer_middleware
