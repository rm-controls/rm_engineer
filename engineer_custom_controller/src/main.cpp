//
// Created by ch on 24-11-23.
//
#include <Eigen/Geometry>
#include "engineer_custom_controller/engineer_custom_controller.h"
#include "engineer_custom_controller/custom_controller_data_sender.h"
#include "engineer_custom_controller/common/data.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "engineer_custom_controller");
  ros::NodeHandle nh("~");
  engineer_custom_controller::EngineerCustomController enngineer_custom_controller(nh);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    // enngineer_custom_controller.read();
    loop_rate.sleep();
  }
}
