//
// Created by ch on 25-6-4.
//

#include "dist_sensor/dist_sensor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dist_sensor");
  ros::NodeHandle nh("~");
  dist_sensor::DistSensor dist_sensor(nh);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    dist_sensor.read(dist_sensor.sensor_l_);
    dist_sensor.read(dist_sensor.sensor_r_);
    dist_sensor.calculateYawAngle();
    loop_rate.sleep();
  }
}
