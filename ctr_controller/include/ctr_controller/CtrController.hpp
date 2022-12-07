#include "alglib/optimization.h"
#include "alglib/stdafx.h"
#include "custom_msg/calculation_acAction.h"
#include "custom_msg/control_values_msg.h"
#include "custom_msg/tip_pos_msg.h"
#include "custom_msg/update_rviz_msg.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <can_msgs/Frame.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <libdlib/control.h>
#include <libdlib/gui_widgets.h>
#include <libdlib/image_transforms.h>
#include <queue>
#include <random>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
using namespace dlib;

using namespace alglib;
namespace ctr_controller {
class CtrController {

public:
  CtrController(ros::NodeHandle &nodeHandle);
  ~CtrController();
  void CostFunction(const real_1d_array &x0, double &fi, void *ptr);

private:
  void CalculateCtrModel();
  void FollowRectangleTrajectory(double length);
  double GenerateTrajectoryWithConstantSpeed(double vel, double sample_time);
  void chatterCallbackControllerInput(
      const custom_msg::control_values_msg::ConstPtr &msg);
  void SetInitialValues();
  void UpdateRviz();
  void EDMD();
  void UpdateEDMD();
  void ApproximateEDMD();
  void VerifyEDMD();
  void CompareResults(real_2d_array inp);
  void MPCStepping();

  ros::Publisher publisher_rviz_;
  ros::Subscriber control_input;
  typedef actionlib::SimpleActionClient<custom_msg::calculation_acAction>
      Client;
  Client client;

  double measured_tip_pos[3];
  std::vector<std::vector<double>> shape_sim;
  double q_manual[6];
  double default_joint_pos[6] = {-0.28, -0.14, 0, 0, 0, 0};
  double q_input[6] = {-0.28, -0.14, 0, 0, 0, 0};
  double q_pos[6] = {-0.28, -0.14, 0, 0, 0, 0};
  double q_input_manual[6] = {-0.28, -0.14, 0, 0, 0, 0};
  double q_simulation[6];
  double q_edmd[6];
  double sim_force[3] = {0.0, 0.0, 0.0};
  bool manual_mode = false;
  bool manual_movement = false;

  double tip_position[3];
  double desired_pos[3] = {0, 0, 0};
  double target_pos[3] = {0, 0, 0};
  std::string mode;
  std::string last_mode;
  std::string last_task;
  std::string CURRENT_MODE;
  std::string CURRENT_TASK;
  std::string CONTROLLER;

  double MAX_STEP_ROTATIONAL = 3 * M_PI / 180;
  double MAX_STEP_TRANSLATIONAL = 0.0001;
  int SAMPLE_FREQ = 40;
  double hard_llimit[3] = {-0.505, -0.251, -0.174};
  double hard_ulimit[3] = {0, 0, 0};
  bool cont_trajectory = true;
  bool motion_control_done = false;
  double end_pos[3];
  double current_pos[3];
  double prev_end_pos[3];
  double a_c[3][3];
  int rect_progress;
  double time_progress;
  double total_time_trajectory;
  double starting_point[3];
  double dt;
  double vel;
  double MPC_Q = 10;
  double MPC_R_ROT = 10;
  double MPC_R_TR = 10;
  int round = 0;
  bool EDMD_MODE = false;

  double e_model[3] = {3.7258e+10, 6.35704e+10, 4.71e+10};
  double g_model[3] = {3.43928e+10, 5.36801e+10, 2.97e+10};

  double tip_disturbed[3];

  std::ofstream stepping_file;

  double A[20][20];
  double B[20][20];
  std::vector<std::vector<double>> U;
  std::vector<std::vector<double>> psi_All;
  const int edmd_stored_size = 250;
  const int edmd_steps = 250;
  const int n_lifting = 12;
  const int n_inputs = 6;
  const int m_length = 18;
  real_2d_array Mat_Final_A;
  real_2d_array Mat_Final_B;
  int approximate_edmd_round = 0;

  const int STATES = 12;
  const int CONTROLS = 6;

  const int HORIZON = 10;
  double q_weight[6] = {1, 1, 1, 0, 0, 0};
  double NOISE = 0;
  int MPC_HORIZON = 10;
};

} // namespace ctr_controller
