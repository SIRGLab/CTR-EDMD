#include "ctr_controller/CtrController.hpp"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
int main(int argc, char **argv) {
  ros::init(argc, argv, "ctr_controller");
  ros::NodeHandle nodeHandle("~");

  ctr_controller::CtrController ctr_controller_(nodeHandle);
  ros::Rate rate(10000);
  ros::spin();
  return 0;
}
