#include <ros/ros.h>
#include "update_rviz/UpdateRviz.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "update_rviz");
  ros::NodeHandle nodeHandle("~");

  update_rviz::UpdateRviz UpdateRviz(nodeHandle);

    // Create a rate
    ros::Rate rate(10000);
    // Use the rate to publish at a fixed rate
    // inside the main function of the program
    while (ros::ok())
    {
        //read_tip_position::ReadTipPosition.readTemperatureSensorData();
        //readTipPosition.PublishPosition();
        ros::spinOnce();
        rate.sleep();
    }

  //ros::spin();
  return 0;
}
