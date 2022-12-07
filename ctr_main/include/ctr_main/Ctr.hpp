#include "custom_msg/calculation_acAction.h"

#include "alglib/optimization.h"
#include "alglib/stdafx.h"
#include "custom_msg/tip_pos_msg.h"
#include <actionlib/client/simple_action_client.h>
#include <can_msgs/Frame.h>
#include <ctr_main/MyParamsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

using namespace alglib;
namespace ctr {

class Ctr {

public:
  Ctr(ros::NodeHandle &nodeHandle);
  ~Ctr();

private:
  void PublishControllerValues();
  void chatterCallbackTipPosition(const custom_msg::tip_pos_msg::ConstPtr &msg);
  void chatterCallbackCan(const can_msgs::Frame msg);
  void MoveMotorsRelative(double t1, double t2, double r1, double r2);
  void MoveMotorsAbsolute(double t1, double t2, double r1, double r2);
  void callbackDynamic(ctr_main::MyParamsConfig &config, uint32_t level);
  double GenerateTrajectoryWithConstantSpeed(double vel, double sample_time);
  typedef actionlib::SimpleActionClient<custom_msg::calculation_acAction>
      Client;

  ros::AsyncSpinner spinner;
  ros::Subscriber sub_tip_pos;
  ros::Subscriber sub_can;

  ros::Publisher publisher_motor_;
  ros::Publisher publisher_controller_;

  ros::CallbackQueue my_queue;

  bool initialization = false;
  bool simulation = false;
  bool limit_switch[4];
  double measured_tip_pos[3];
  double quaternion[4];
  double q_manual[6];
  double default_joint_pos[6] = {-0.28, -0.14, -0.0945, 0, 0, 0};
  double q_input[6] = {-0.28, -0.14, -0.0945, 0, 0, 0};
  double read_position[6] = {0, 0, 0, 0, 0, 0};
  bool manual_mode = false;
  bool manual_movement = false;
  bool paused = false;
  double JacobianModelBased[3][6];
  double JacobianDataDriven[3][6];
  double tip_position[3];
  bool valid_measurement = false;
  std::string mode;
  std::string task;
  std::string controller;
  std::string last_mode;

  double LEARNING_FACTOR_ROTATIONAL = 0.1;
  double LEARNING_FACTOR_TRANSLATIONAL = 0.1;
  double MAX_STEP_ROTATIONAL = 5 * M_PI / 180;
  double MAX_STEP_TRANSLATIONAL = 0.00075;
  double VELOCITY = 0.005;
  double K_P_TRANSLATIONAL = 0.1;
  double K_P_ROTATIONAL = 0.1;
  int SAMPLE_FREQ = 20;
  int MODEL_JAC_UPDATE = 1;
  int MAX_BUFFER_TRANSLATIONAL = 100;
  int MOVING_AVG_TRANSLATIONAL = 5;
  int MAX_BUFFER_ROTATIONAL = 100;
  int MOVING_AVG_ROTATIONAL = 5;
  double MPC_Q = 10;
  double MPC_R_ROT = 10;
  double MPC_R_TR = 10;
  double NOISE = 0;
  int MPC_HORIZON = 5;
  double GRID_SIZE = 0.001;
  double MOVE_IN_DIST = 0.001;

  double hard_llimit[3] = {-0.505, -0.251, -0.174};
  double hard_ulimit[3] = {0, 0, 0};
  bool cont_trajectory = false;
  double end_pos[3];
  double current_pos[3];
  double prev_end_pos[3];
  double a_c[3][3];
  double ptp_target[3];
  real_2d_array pseudoJ;

  std::ofstream tip_pos_file;
};

} // namespace ctr
