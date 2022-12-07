#include "ctr_main/Ctr.hpp"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "ctr_main");
  ros::NodeHandle nodeHandle("~");

  ctr::Ctr ctr_(nodeHandle);
  return 0;
}
