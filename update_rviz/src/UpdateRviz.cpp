#include "update_rviz/UpdateRviz.hpp"
// STD
#include "custom_msg/update_rviz_msg.h"
#include <sstream>
#include <std_msgs/String.h>
#include <string>
namespace update_rviz {
UpdateRviz::UpdateRviz(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {
  sub_command_controller =
      nodeHandle.subscribe("/ctr_controller/update_rviz", 10,
                           &UpdateRviz::chatterCallbackShape, this);
  vis_publ = nodeHandle.advertise<visualization_msgs::Marker>(
      "visualization_marker", 0);

  std::cout << "UpdateRviz Started\n";
  ros::Rate loop_rate(1);
}

UpdateRviz::~UpdateRviz() {}
void UpdateRviz::chatterCallbackShape(
    const custom_msg::update_rviz_msg::ConstPtr &msg) {
  double q[6];
  double q_0[6] = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 6; i++)
    q[i] = msg->q[i];
  int i = 1;
  while (i < 251) {
    string link_name = "snake_body_";
    link_name += std::to_string(i);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "base_link";
    static_transformStamped.child_frame_id = link_name.c_str();

    if (i < msg->n_points) {
      static_transformStamped.transform.translation.x = msg->x[i] * 10;
      static_transformStamped.transform.translation.y = msg->y[i] * 10;
      static_transformStamped.transform.translation.z = msg->z[i] * 10;
    } else {
      static_transformStamped.transform.translation.x = 0;
      static_transformStamped.transform.translation.y = 0;
      static_transformStamped.transform.translation.z = 0;
    }
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
    i++;
  }
  UpdateMarker("marker_tip", msg->x[msg->n_points - 1],
               msg->y[msg->n_points - 1], msg->z[msg->n_points - 1]);
  UpdateMarker("marker_desired", msg->desired[0], msg->desired[1],
               msg->desired[2]);
  UpdateMarker("marker_target", msg->target[0], msg->target[1], msg->target[2]);
  UpdateMarker("marker_measured", msg->measured[0], msg->measured[1],
               msg->measured[2]);

  for (int j = 0; j < 3; j++) {
    double step_size = ((l[j] + q_0[j] + q[j]) - l_k[j]) / 15;
    // STRAIGHT PART
    for (int i = 1; i < 16; i++) {
      string link_name = "tube_";
      link_name += std::to_string(j + 1);
      link_name += "_";
      link_name += to_string(i);

      static tf2_ros::StaticTransformBroadcaster static_broadcaster;
      geometry_msgs::TransformStamped static_transformStamped;

      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = "base_link";
      static_transformStamped.child_frame_id = link_name.c_str();

      if ((l[j] + q_0[j] + q[j]) > l_k[j]) {
        static_transformStamped.transform.translation.x = 0;
        static_transformStamped.transform.translation.y = 0;
        static_transformStamped.transform.translation.z = step_size * i * 10;

      } else {
        step_size = 0;
        static_transformStamped.transform.translation.x = 0;
        static_transformStamped.transform.translation.y = 0;
        static_transformStamped.transform.translation.z = 0;
      }
      tf2::Quaternion quat;
      quat.setRPY(0, 0, 0);
      static_transformStamped.transform.rotation.x = quat.x();
      static_transformStamped.transform.rotation.y = quat.y();
      static_transformStamped.transform.rotation.z = quat.z();
      static_transformStamped.transform.rotation.w = quat.w();
      static_broadcaster.sendTransform(static_transformStamped);
    }
    // CURVED PART
    double curved = 0;
    if ((l[j] + q_0[j] + q[j]) > l_k[j])
      curved = l_k[j];
    else
      curved = (l[j] + q_0[j] + q[j]);

    double r = 1 / UX[j];
    double angle = curved / r;
    for (int i = 16; i < 31; i++) {
      string link_name = "tube_";
      link_name += std::to_string(j + 1);
      link_name += "_";
      link_name += to_string(i);

      static tf2_ros::StaticTransformBroadcaster static_broadcaster;
      geometry_msgs::TransformStamped static_transformStamped;

      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = "base_link";
      static_transformStamped.child_frame_id = link_name.c_str();
      double curve_combined =
          -r * cos(angle / 15) * 10 + r * cos(angle / 15 * (i - 15)) * 10;
      if (curve_combined != 0) {
        static_transformStamped.transform.translation.x =
            -curve_combined * sin(q[3 + j] + q_0[3 + j]);
        static_transformStamped.transform.translation.y =
            curve_combined * cos(q[3 + j] + q_0[3 + j]);
        static_transformStamped.transform.translation.z =
            (step_size * 15 + r * sin(angle / 15 * (i - 15))) * 10;
      } else {
        static_transformStamped.transform.translation.x = 0;
        static_transformStamped.transform.translation.y = 0;
        static_transformStamped.transform.translation.z = 0;
      }

      tf2::Quaternion quat;
      quat.setRPY(0, 0, 0);
      static_transformStamped.transform.rotation.x = quat.x();
      static_transformStamped.transform.rotation.y = quat.y();
      static_transformStamped.transform.rotation.z = quat.z();
      static_transformStamped.transform.rotation.w = quat.w();
      static_broadcaster.sendTransform(static_transformStamped);
    }
  }
}
void UpdateRviz::UpdateMarker(std::string marker_name, double x, double y,
                              double z) {
  if (isnan(x) || isnan(y) || isnan(z) || isinf(x) || isinf(y) || isinf(z))
    return;
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "base_link";
  static_transformStamped.child_frame_id = marker_name.c_str();

  static_transformStamped.transform.translation.x = x * 10;
  static_transformStamped.transform.translation.y = y * 10;
  static_transformStamped.transform.translation.z = z * 10;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
}
} // namespace update_rviz
