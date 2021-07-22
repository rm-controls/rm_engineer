//
// Created by astro on 2021/4/25.
//

#include "engineer_middleware/middleware.h"
#include <unistd.h>
namespace engineer_middleware {
Middleware::Middleware(ros::NodeHandle &nh) :
    nh_(nh),
    as_(nh_, "move_arm", [this](auto &&PH1) { executeCB(std::forward<decltype(PH1)>(PH1)); }, false),
    arm_group_(moveit::planning_interface::MoveGroupInterface("engineer_arm")),
    hand_group_(moveit::planning_interface::MoveGroupInterface("engineer_hand")),
    chassis_interface_(nh, tf_),
    card_pub_(nh.advertise<std_msgs::Float64>("/controllers/card_position_controller/command", 10)),
    gimbal_pub_(nh.advertise<rm_msgs::GimbalCmd>("/cmd_gimbal", 10)),
    tf_listener_(tf_),
    is_middleware_control_(false) {
  if (nh.hasParam("steps_list")) {
    XmlRpc::XmlRpcValue xml_value;
    nh.getParam("steps_list", xml_value);
    ROS_ASSERT(xml_value.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = xml_value.begin(); it != xml_value.end(); ++it) {
      step_queues_.insert(std::make_pair(std::stoi(it->first, nullptr, 0), StepQueue(
          it->second, tf_, arm_group_, hand_group_, chassis_interface_, card_pub_, gimbal_pub_)));
    }
  } else
    ROS_ERROR("no steps list define in yaml");
  as_.start();
}

}