//
// Created by qiayuan on 5/30/21.
//

#ifndef ENGINEER_MIDDLEWARE_CHASSIS_INTERFACE_H_
#define ENGINEER_MIDDLEWARE_CHASSIS_INTERFACE_H_

#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

namespace engineer_middleware {
class ChassisInterface {
 public:
  bool setGoal(const geometry_msgs::PoseStamped &pose);
  void update();
  double getPosError();
  double getYawError();
 private:
  control_toolbox::Pid pid_x_, pid_y_, pid_w_;
};
}
#endif //ENGINEER_MIDDLEWARE_CHASSIS_INTERFACE_H_
