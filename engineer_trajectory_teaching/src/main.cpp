//
// Created by ch on 25-7-7.
//
#include "engineer_trajectory_teaching/engineer_trajectory_teaching.h"

using namespace engineer_trajectory_teaching;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "engineer_trajectory_teaching");
  ros::NodeHandle nh("~");
  engineer_trajectory_teaching::EngineerTrajectoryTeaching engineer_trajectory_teaching(nh);
  ros::Rate loop_rate(100);
  ros::MultiThreadedSpinner spinner(3);
  while (ros::ok())
  {
    spinner.spin();
    loop_rate.sleep();
  }
  return 0;
}
