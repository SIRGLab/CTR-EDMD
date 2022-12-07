#pragma once

// ROS
#include "custom_msg/update_rviz_msg.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <can_msgs/Frame.h>
#include <math.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
using namespace std;
namespace update_rviz {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class UpdateRviz {
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  UpdateRviz(ros::NodeHandle &nodeHandle);
  /*!
   * Destructor.
   */
  virtual ~UpdateRviz();

private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  //! ROS node handle.
  void chatterCallbackShape(const custom_msg::update_rviz_msg::ConstPtr &msg);
  void UpdateMarker(std::string marker_name, double x, double y, double z);
  void UpdateMarkerManipulability(std::string marker_name, double x, double y,
                                  double z, double quatern[4], double scale[3]);

  ros::NodeHandle &nodeHandle_;

  ros::Subscriber sub_command_controller;
  ros::Publisher vis_publ;
  double UX[3] = {14, 10, 3.5};
  double l[3] = {0.5050, 0.251, 0.174};
  double l_k[3] = {0.1454, 0.1471, 0.134};
};

} // namespace update_rviz
