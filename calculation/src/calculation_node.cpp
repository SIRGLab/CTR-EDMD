#include "calculation/Calculation.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "calculation");
  ros::NodeHandle nodeHandle("~");

  calculation::Calculation cal(nodeHandle);
  ros::Rate rate(10000);

  ros::spin();
  return 0;
}
