#include "ctr_main/Ctr.hpp"
#include "ctr_main/MyParamsConfig.h"
#include "custom_msg/calculation_acAction.h"
#include "custom_msg/control_values_msg.h"
#include "custom_msg/motor_commands_msg.h"
#include "custom_msg/tip_pos_msg.h"
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <vector>

namespace ctr {

Ctr::Ctr(ros::NodeHandle &nodeHandle) : spinner(0) {
  ROS_INFO("Init Started");
  publisher_controller_ =
      nodeHandle.advertise<custom_msg::control_values_msg>("control_values", 1);
  dynamic_reconfigure::Server<ctr_main::MyParamsConfig> server;
  dynamic_reconfigure::Server<ctr_main::MyParamsConfig>::CallbackType f;
  f = boost::bind(&Ctr::callbackDynamic, this, _1, _2);
  server.setCallback(f);
  ctr_main::MyParamsConfig config;
  tip_pos_file.open("tip_pos.csv");
  // boost::thread thread_b(do_stuff, &rate_b);
  last_mode = "NA";

  ros::Rate r(SAMPLE_FREQ); // 20 hz
  while (ros::ok()) {
    ros::spinOnce();
    PublishControllerValues();
  }
}
Ctr::~Ctr() {}

void Ctr::PublishControllerValues() {
  custom_msg::control_values_msg msg;
  msg.mode = mode;
  msg.task = task;
  msg.controller = controller;

  msg.noise = NOISE;

  msg.manual_t1 = q_manual[0];
  msg.manual_t2 = q_manual[1];
  msg.manual_t3 = q_manual[2];
  msg.manual_r1 = q_manual[3];
  msg.manual_r2 = q_manual[4];
  msg.manual_r3 = q_manual[5];

  msg.max_rotational = MAX_STEP_ROTATIONAL;
  msg.max_translational = MAX_STEP_TRANSLATIONAL;

  msg.velocity = VELOCITY;

  msg.mpc_q = MPC_Q;
  msg.mpc_r_rot = MPC_R_ROT;
  msg.mpc_r_tr = MPC_R_TR;

  publisher_controller_.publish(msg);
}

void Ctr::callbackDynamic(ctr_main::MyParamsConfig &config, uint32_t level) {
  q_manual[0] = config.beta_1;
  q_manual[1] = config.beta_2;
  q_manual[2] = config.beta_3;
  q_manual[3] = config.alpha_1;
  q_manual[4] = config.alpha_2;
  q_manual[5] = config.alpha_3;

  MAX_STEP_ROTATIONAL = config.max_rotational;
  MAX_STEP_TRANSLATIONAL = config.max_translational;

  VELOCITY = config.velocity / 1000;

  if (config.mode == 0)
    mode = "MANUAL_MODE";
  else if (config.mode == 1)
    mode = "KOOPMAN";

  if (config.task == 0)
    task = "NONE";
  else if (config.task == 1)
    task = "INITIAL_LEARNING";
  else if (config.task == 2)
    task = "SQUARE";

  controller = "MPC";

  NOISE = config.noise;
  MPC_Q = config.mpc_q;
  MPC_R_ROT = config.mpc_r_rot;
  MPC_R_TR = config.mpc_r_tr;
}

} // namespace ctr