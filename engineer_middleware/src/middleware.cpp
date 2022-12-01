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

//
// Created by astro on 2021/4/25.
//

#include "engineer_middleware/middleware.h"
#include <unistd.h>
namespace engineer_middleware
{
Middleware::Middleware(ros::NodeHandle& nh)
  : nh_(nh)
  , as_(
        nh_, "move_steps", [this](auto&& PH1) { executeCB(std::forward<decltype(PH1)>(PH1)); }, false)
  , arm_group_(moveit::planning_interface::MoveGroupInterface("engineer_arm"))
  , chassis_interface_(nh, tf_)
  , hand_pub_(nh.advertise<std_msgs::Float64>("/controllers/hand_controller/command", 10))
  , card_pub_(nh.advertise<std_msgs::Float64>("/controllers/card_controller/command", 10))
  , gimbal_pub_(nh.advertise<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10))
  , gpio_pub_(nh.advertise<rm_msgs::GpioData>("/controllers/gpio_controller/command", 10))
  , tf_listener_(tf_)
  , is_middleware_control_(false)
  , target_sub_(nh.subscribe<geometry_msgs::TwistStamped>("/position", 10, &Middleware::targetPosDataCallback, this))
{
  if (nh.hasParam("steps_list") && nh.hasParam("scenes_list"))
  {
    XmlRpc::XmlRpcValue steps_list;
    XmlRpc::XmlRpcValue scenes_list;
    nh.getParam("steps_list", steps_list);
    nh.getParam("scenes_list", scenes_list);
    ROS_ASSERT(steps_list.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct);
    ROS_ASSERT(scenes_list.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = steps_list.begin(); it != steps_list.end(); ++it)
    {
      step_queues_.insert(
          std::make_pair(it->first, StepQueue(it->second, scenes_list, tf_, arm_group_, chassis_interface_, hand_pub_,
                                              card_pub_, gimbal_pub_, gpio_pub_)));
    }
  }
  else
    ROS_ERROR("no steps list define in yaml");
  as_.start();
}

}  // namespace engineer_middleware
