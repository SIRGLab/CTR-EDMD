#include "ctr_controller/CtrController.hpp"
#include "custom_msg/calculation_acAction.h"
#include "custom_msg/control_values_msg.h"
#include "custom_msg/motor_commands_msg.h"
#include "custom_msg/tip_pos_msg.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

using Eigen::MatrixXd;
using namespace Eigen;
int EDMD_COUNT = 1;
int edmd_update_round = 0;
int position_state = 0;
std::string TASK;
namespace ctr_controller {
CtrController::CtrController(ros::NodeHandle &nodeHandle)
    : client("calculation", true) {
  last_mode = "NA";
  SAMPLE_FREQ = 40;
  publisher_rviz_ =
      nodeHandle.advertise<custom_msg::update_rviz_msg>("update_rviz", 10000);
  control_input = nodeHandle.subscribe(
      "/ctr_main/control_values", 1,
      &CtrController::chatterCallbackControllerInput, this);
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
}
CtrController::~CtrController() {}
// --------------------------------------------------------------------------------------
// RECEIVE CONTROLLER VALUES FROM CTR_MAIN NODE -> DO THE ACTION ACCORDINGLY
// --------------------------------------------------------------------------------------
void CtrController::chatterCallbackControllerInput(
    const custom_msg::control_values_msg::ConstPtr &msg) {
  // UPDATE MEASURED VALUES
  measured_tip_pos[0] = msg->tip_x;
  measured_tip_pos[1] = msg->tip_y;
  measured_tip_pos[2] = msg->tip_z;
  CURRENT_MODE = msg->mode;
  CURRENT_TASK = msg->task;
  CONTROLLER = msg->controller;

  MPC_Q = msg->mpc_q;
  MPC_R_ROT = msg->mpc_r_rot;
  MPC_R_TR = msg->mpc_r_tr;

  NOISE = msg->noise;
  q_input[0] = msg->t1;
  q_input[1] = msg->t2;
  q_input[2] = msg->t3;
  q_input[3] = msg->r1;
  q_input[4] = msg->r2;
  q_input[5] = msg->r3;

  q_input_manual[0] = msg->manual_t1;
  q_input_manual[1] = msg->manual_t2;
  q_input_manual[2] = msg->manual_t3;
  q_input_manual[3] = msg->manual_r1;
  q_input_manual[4] = msg->manual_r2;
  q_input_manual[5] = msg->manual_r3;

  MAX_STEP_TRANSLATIONAL = msg->max_translational;
  MAX_STEP_ROTATIONAL = msg->max_rotational;

  vel = msg->velocity;
  TASK = msg->task;

  if (msg->mode == "KOOPMAN") {
    EDMD_MODE = true;
  } else
    EDMD_MODE = false;
  if (msg->mode != last_mode || msg->task != last_task) {
    SetInitialValues();
  } else if (msg->mode == "MANUAL_MODE") {
    for (int i = 0; i < 6; i++)
      q_simulation[i] = q_input_manual[i];
    CalculateCtrModel();
  } else if (msg->task == "SQUARE" && !motion_control_done) {
    FollowRectangleTrajectory(0.02);
  } else if (msg->task == "INITIAL_LEARNING") {
    if (msg->mode == "KOOPMAN") {
      ApproximateEDMD();
      EDMD_COUNT = 0;
    }
  }

  last_mode = msg->mode;
  last_task = msg->task;
}

// --------------------------------------------------------------------------------------
// SET INITIAL VALUES
// --------------------------------------------------------------------------------------
void CtrController::SetInitialValues() {
  CalculateCtrModel();
  motion_control_done = false;
  rect_progress = 0;
  time_progress = 0;
  round = 0;
  for (int j = 0; j < 6; j++) {
    q_edmd[j] = q_input_manual[j];
    q_simulation[j] = q_input_manual[j];
  }
  approximate_edmd_round = 0;
  if (TASK == "INITIAL_LEARNING")
    EDMD();
  for (int i = 0; i < 3; i++)
    tip_position[i] = shape_sim[shape_sim.size() - 1][i];
}

// --------------------------------------------------------------------------------------
// REQUEST CTR SHAPE FROM CALCULATION NODE, SET MODEL PARAMETERS
// --------------------------------------------------------------------------------------
void CtrController::CalculateCtrModel() {
  custom_msg::calculation_acGoal goal;
  // Fill in goal here
  goal.q.insert(goal.q.begin(), q_simulation, q_simulation + 6);

  goal.e.insert(goal.e.end(), e_model, e_model + 3);
  goal.g.insert(goal.g.end(), g_model, g_model + 3);
  goal.force.insert(goal.force.begin(), sim_force, sim_force + 3);
  client.sendGoal(goal);

  bool finished_before_timeout = client.waitForResult(ros::Duration(15.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = client.getState();
    custom_msg::calculation_acResultConstPtr result = client.getResult();
    tip_position[0] = result->x[result->x.size() - 1];
    tip_position[1] = result->y[result->y.size() - 1];
    tip_position[2] = result->z[result->z.size() - 1];

    shape_sim.clear();
    for (int i = 0; i < result->x.size(); i++) {
      std::vector<double> temp;
      temp.push_back(result->x[i]);
      temp.push_back(result->y[i]);
      temp.push_back(result->z[i]);
      shape_sim.push_back(temp);
    }
    UpdateRviz();
  } else
    ROS_INFO("Action did not finish before the time out.");
}

// --------------------------------------------------------------------------------------
// REQUEST SHAPE, TUBES AND TARGET UPDATE IN RVIZ
// --------------------------------------------------------------------------------------
void CtrController::UpdateRviz() {
  custom_msg::update_rviz_msg msg;
  msg.n_points = shape_sim.size();
  for (int i = 0; i < shape_sim.size(); i++) {
    msg.x.push_back(shape_sim[i][0]);
    msg.y.push_back(shape_sim[i][1]);
    msg.z.push_back(shape_sim[i][2]);
  }
  for (int i = 0; i < 6; i++) {
    msg.q.push_back(q_simulation[i]);
  }
  for (int i = 0; i < 3; i++) {
    msg.desired.push_back(desired_pos[i]);
    msg.target.push_back(target_pos[i]);
    msg.measured.push_back(measured_tip_pos[i]);
  }

  publisher_rviz_.publish(msg);
}

// --------------------------------------------------------------------------------------
// STEPPING THROUGH A SQUARE TRAJECTORY
// --------------------------------------------------------------------------------------
void CtrController::FollowRectangleTrajectory(double length) {
  if (time_progress >= total_time_trajectory) {
    time_progress = 0;
    round = 0;

    // ALL THE SIDES OF THE SQUARE ARE DONE
    if (rect_progress == 4)
      motion_control_done = true;
    stepping_file << std::flush;
  }

  if (time_progress == 0) {
    if (rect_progress == 0) {
      // 1ST SIDE OF THE RECTANGLE
      stepping_file.close();
      stepping_file.open("stepping.csv");

      starting_point[0] = tip_position[0];
      starting_point[1] = tip_position[1];
      starting_point[2] = tip_position[2];

      prev_end_pos[0] = tip_position[0];
      prev_end_pos[1] = tip_position[1];
      prev_end_pos[2] = tip_position[2];

      end_pos[0] = starting_point[0];
      end_pos[1] = starting_point[1];
      end_pos[2] = starting_point[2] + length;
    } else if (rect_progress == 1) {
      // 2ND SIDE OF THE RECTANGLE
      prev_end_pos[0] = starting_point[0];
      prev_end_pos[1] = starting_point[1];
      prev_end_pos[2] = starting_point[2] + length;
      end_pos[0] = starting_point[0] - length;
      end_pos[1] = starting_point[1];
      end_pos[2] = starting_point[2] + length;
    } else if (rect_progress == 2) {
      // 3RD SIDE OF THE RECTANGLE
      prev_end_pos[0] = starting_point[0] - length;
      prev_end_pos[1] = starting_point[1];
      prev_end_pos[2] = starting_point[2] + length;
      end_pos[0] = starting_point[0] - length;
      end_pos[1] = starting_point[1];
      end_pos[2] = starting_point[2];
    } else if (rect_progress == 3) {
      // 4TH SIDE OF THE RECTANGLE
      prev_end_pos[0] = starting_point[0] - length;
      prev_end_pos[1] = starting_point[1];
      prev_end_pos[2] = starting_point[2];
      end_pos[0] = starting_point[0];
      end_pos[1] = starting_point[1];
      end_pos[2] = starting_point[2];
    }

    total_time_trajectory = GenerateTrajectoryWithConstantSpeed(vel, dt);
    dt = 1 / (double)SAMPLE_FREQ;
    rect_progress++;
  }

  // DO IT UNTIL ALL 4 SIDES ARE DONE
  if (!motion_control_done) {
    if (CURRENT_MODE == "KOOPMAN") {
      if (time_progress != 0) {
        if (edmd_update_round % 2 == 0) {
          UpdateEDMD();
          edmd_update_round = 0;
        } else
          edmd_update_round++;
      }
      MPCStepping();
      EDMD_COUNT++;
    }
  }
  round++;
}

// --------------------------------------------------------------------------------------
// INITIALIZE VALUES FOR EDMD
// --------------------------------------------------------------------------------------
void CtrController::EDMD() {
  total_time_trajectory = 3;
  dt = 1 / (double)SAMPLE_FREQ;
  for (int i = 0; i < 6; i++)
    q_edmd[i] = q_input_manual[i];

  // INITIALIZE EDMD MATRICES
  double A[edmd_stored_size][3];
  for (int i = 0; i < edmd_stored_size; i++) {
    for (int j = 0; j < 3; j++)
      A[i][j] = shape_sim[shape_sim.size() - 1][j];
    std::vector<double> temp;
    for (int j = 0; j < n_inputs; j++)
      temp.push_back(0);
    U.push_back(temp);
  }

  // DEFINE psi with lifting functions
  for (int i = 0; i < edmd_stored_size; i++) {
    std::vector<double> temp;
    temp.push_back(A[i][0]);
    temp.push_back(A[i][1]);
    temp.push_back(A[i][2]);

    temp.push_back(A[i][0] * A[i][0] * A[i][1]);
    temp.push_back(A[i][0] * A[i][0] * A[i][2]);
    temp.push_back(A[i][1] * A[i][1] * A[i][0]);
    temp.push_back(A[i][1] * A[i][1] * A[i][2]);
    temp.push_back(A[i][2] * A[i][2] * A[i][0]);
    temp.push_back(A[i][2] * A[i][2] * A[i][1]);

    psi_All.push_back(temp);
  }

  real_2d_array Mat_A;
  Mat_A.setlength(edmd_steps - 1, m_length);
  real_2d_array Mat_B;
  Mat_B.setlength(edmd_steps - 1, m_length);
  for (int i = 0; i < edmd_steps - 1; i++) {
    for (int j = 0; j < n_lifting; j++) {
      Mat_A[i][j] = psi_All[i + 1][j];
      Mat_B[i][j] = psi_All[i][j];
    }
    if (i != edmd_steps - 2) {
      for (int j = 0; j < n_inputs; j++) {
        Mat_A[i][j + n_lifting] = U[i][j];
        Mat_B[i][j + n_lifting] = U[i][j];
      }
    }
  }
}

// --------------------------------------------------------------------------------------
// UPDATE EDMD VALUES BASED ON LAST INPUTS AND STATES
// --------------------------------------------------------------------------------------
void CtrController::UpdateEDMD() {
  double x, y, z;

  for (int i = 0; i < 3; i++) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dis(0.000, NOISE);
    double rand = dis(gen);
    tip_disturbed[i] = tip_position[i] + rand;
  }
  x = tip_disturbed[0];
  y = tip_disturbed[1];
  z = tip_disturbed[2];

  std::vector<double> temp;

  temp.push_back(x);
  temp.push_back(y);
  temp.push_back(z);

  temp.push_back(x * x);
  temp.push_back(y * y);
  temp.push_back(z * z);

  temp.push_back(x * x * y);
  temp.push_back(x * x * z);
  temp.push_back(y * y * x);
  temp.push_back(y * y * z);
  temp.push_back(z * z * x);
  temp.push_back(z * z * y);

  psi_All.erase(psi_All.end());
  psi_All.insert(psi_All.begin(), temp);
  U.erase(U.end());
  std::vector<double> temp_inp;

  temp_inp.push_back(q_edmd[0]);
  temp_inp.push_back(q_edmd[1]);
  temp_inp.push_back(q_edmd[2]);
  temp_inp.push_back(q_edmd[3]);
  temp_inp.push_back(q_edmd[4]);
  temp_inp.push_back(q_edmd[5]);

  U.insert(U.begin(), temp_inp);

  real_2d_array Mat_A;
  Mat_A.setlength(edmd_steps - 1, m_length);
  real_2d_array Mat_B;
  Mat_B.setlength(edmd_steps - 1, m_length);
  for (int i = 0; i < edmd_steps - 1; i++) {
    for (int j = 0; j < n_lifting; j++) {
      Mat_A[i][j] = psi_All[i + 1][j];
      Mat_B[i][j] = psi_All[i][j];
    }
    if (i != edmd_steps - 1) {
      for (int j = 0; j < n_inputs; j++) {
        Mat_A[i][j + n_lifting] = U[i][j];
        Mat_B[i][j + n_lifting] = U[i][j];
      }
    }
  }

  MatrixXd test(edmd_steps - 1, m_length);
  for (int i = 0; i < edmd_steps - 1; i++) {
    for (int j = 0; j < m_length; j++) {
      test(i, j) = Mat_A[i][j];
    }
  }

  Eigen::MatrixXd pinv = test.completeOrthogonalDecomposition().pseudoInverse();
  real_2d_array pseudoInvMat;
  pseudoInvMat.setlength(m_length, edmd_steps - 1);
  for (int i = 0; i < m_length; i++) {
    for (int j = 0; j < edmd_steps - 1; j++) {
      pseudoInvMat(i, j) = pinv(i, j);
    }
  }

  real_2d_array Mat_res;
  Mat_res.setlength(m_length, m_length);

  rmatrixgemm(m_length, m_length, edmd_steps - 1, 1, pseudoInvMat, 0, 0, 0,
              Mat_B, 0, 0, 0, 0, Mat_res, 0, 0);

  real_2d_array Mat_res_T;
  Mat_res_T.setlength(m_length, m_length);
  for (int i = 0; i < m_length; i++) {
    for (int j = 0; j < m_length; j++) {
      Mat_res_T[i][j] = Mat_res[j][i];
    }
  }

  Mat_Final_A.setlength(n_lifting, n_lifting);
  Mat_Final_B.setlength(n_lifting, n_inputs);

  for (int i = 0; i < n_lifting; i++) {
    for (int j = 0; j < n_lifting; j++) {
      Mat_Final_A[i][j] = Mat_res_T[i][j];
    }
    for (int j = 0; j < n_inputs; j++) {
      Mat_Final_B[i][j] = Mat_res_T[i][j + n_lifting];
    }
  }
}

bool already_verified = false;
// --------------------------------------------------------------------------------------
// INITIAL EDMD LEARNING -> LEARN THE DYNAMICS OF THE SYSTEM
// --------------------------------------------------------------------------------------
void CtrController::ApproximateEDMD() {

  // REPEAT IT UNTIL THE REQUESTED TIME (TOTAL_TIME_TRAJECTORY)
  if (time_progress <= total_time_trajectory) {
    std::cout << "Learning phase " << time_progress << " / "
              << total_time_trajectory << "\n";
    // FILL UP MATRICES
    real_2d_array Mat_A;
    Mat_A.setlength(edmd_steps - 1, m_length);
    real_2d_array Mat_B;
    Mat_B.setlength(edmd_steps - 1, m_length);
    for (int i = 0; i < edmd_steps - 1; i++) {
      for (int j = 0; j < n_lifting; j++) {
        Mat_A[i][j] = psi_All[i + 1][j];
        Mat_B[i][j] = psi_All[i][j];
      }
      if (i != edmd_steps - 1) {
        for (int j = 0; j < n_inputs; j++) {
          Mat_A[i][j + n_lifting] = U[i][j];
          Mat_B[i][j + n_lifting] = U[i][j];
        }
      }
    }

    MatrixXd test(edmd_steps - 1, m_length);
    for (int i = 0; i < edmd_steps - 1; i++) {
      for (int j = 0; j < m_length; j++) {
        test(i, j) = Mat_A[i][j];
      }
    }

    Eigen::MatrixXd pinv =
        test.completeOrthogonalDecomposition().pseudoInverse();
    real_2d_array pseudoInvMat;
    pseudoInvMat.setlength(m_length, edmd_steps - 1);
    for (int i = 0; i < m_length; i++) {
      for (int j = 0; j < edmd_steps - 1; j++) {
        pseudoInvMat(i, j) = pinv(i, j);
      }
    }

    real_2d_array Mat_res;
    Mat_res.setlength(m_length, m_length);

    rmatrixgemm(m_length, m_length, edmd_steps - 1, 1, pseudoInvMat, 0, 0, 0,
                Mat_B, 0, 0, 0, 0, Mat_res, 0, 0);

    real_2d_array Mat_res_T;
    Mat_res_T.setlength(m_length, m_length);
    for (int i = 0; i < m_length; i++) {
      for (int j = 0; j < m_length; j++) {
        Mat_res_T[i][j] = Mat_res[j][i];
      }
    }

    // GENERATE RANDOM JOINT INPUTS
    std::vector<double> gen_q(6);
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int j = 0; j < 3; j++) {
      std::uniform_real_distribution<> dis(-0.001, 0.001);
      gen_q[j] = dis(gen);
    }
    for (int j = 3; j < 6; j++) {
      std::uniform_real_distribution<> dis(-0.002, 0.002);
      gen_q[j] = dis(gen);
    }

    if (approximate_edmd_round % 15 == 0)
      q_edmd[0] += gen_q[0];
    if (approximate_edmd_round % 15 == 1)
      q_edmd[1] += gen_q[1];
    if (approximate_edmd_round % 15 == 2)
      q_edmd[3] += gen_q[3];
    if (approximate_edmd_round % 15 == 3)
      q_edmd[4] += gen_q[4];
    if (approximate_edmd_round % 15 == 4)
      q_edmd[2] += gen_q[2];
    if (approximate_edmd_round % 15 == 5)
      q_edmd[5] += gen_q[5];

    if (approximate_edmd_round % 15 == 6) {
      q_edmd[0] += gen_q[0];
      q_edmd[1] += gen_q[1];
    }
    if (approximate_edmd_round % 15 == 7) {
      q_edmd[1] += gen_q[1];
      q_edmd[2] += gen_q[2];
    }
    if (approximate_edmd_round % 15 == 8) {
      q_edmd[3] += gen_q[3];
      q_edmd[4] += gen_q[4];
    }
    if (approximate_edmd_round % 15 == 9) {
      q_edmd[4] += gen_q[4];
      q_edmd[5] += gen_q[5];
    }
    if (approximate_edmd_round % 15 == 10) {
      q_edmd[0] += gen_q[0];
      q_edmd[3] += gen_q[3];
    }
    if (approximate_edmd_round % 15 == 11) {
      q_edmd[1] += gen_q[1];
      q_edmd[4] += gen_q[4];
    }
    if (approximate_edmd_round % 15 == 12) {
      q_edmd[2] += gen_q[2];
      q_edmd[5] += gen_q[5];
    }
    if (approximate_edmd_round % 15 == 13) {
      q_edmd[0] += gen_q[0];
      q_edmd[1] += gen_q[1];
      q_edmd[3] += gen_q[3];
      q_edmd[4] += gen_q[4];
    }
    if (approximate_edmd_round % 15 == 14) {
      q_edmd[0] += gen_q[0];
      q_edmd[1] += gen_q[1];
      q_edmd[2] += gen_q[2];
      q_edmd[3] += gen_q[3];
      q_edmd[4] += gen_q[4];
      q_edmd[5] += gen_q[5];
    }
    approximate_edmd_round++;

    // STORE A,B MATRIX VALUES
    Mat_Final_A.setlength(n_lifting, n_lifting);
    Mat_Final_B.setlength(n_lifting, n_inputs);

    for (int i = 0; i < n_lifting; i++) {
      for (int j = 0; j < n_lifting; j++) {
        Mat_Final_A[i][j] = Mat_res_T[i][j];
      }
      for (int j = 0; j < n_inputs; j++) {
        Mat_Final_B[i][j] = Mat_res_T[i][j + n_lifting];
      }
    }
    time_progress += dt;

    U.erase(U.end());
    std::vector<double> temp_inp;
    temp_inp.push_back(q_edmd[0]);
    temp_inp.push_back(q_edmd[1]);
    temp_inp.push_back(q_edmd[2]);
    temp_inp.push_back(q_edmd[3]);
    temp_inp.push_back(q_edmd[4]);
    temp_inp.push_back(q_edmd[5]);
    U.insert(U.begin(), temp_inp);

    real_2d_array temp_A;
    temp_A.setlength(n_lifting, 1);
    real_2d_array temp_B;
    temp_B.setlength(n_lifting, 1);
    real_2d_array current_state;
    current_state.setlength(n_lifting, 1);
    real_2d_array current_input;
    current_input.setlength(n_inputs, 1);
    for (int i = 0; i < n_lifting; i++)
      current_state[i][0] = psi_All[0][i];
    for (int i = 0; i < n_inputs; i++)
      current_input[i][0] = temp_inp[i];

    rmatrixgemm(n_lifting, 1, n_lifting, 1, Mat_Final_A, 0, 0, 0, current_state,
                0, 0, 0, 0, temp_A, 0, 0);

    rmatrixgemm(n_lifting, 1, n_inputs, 1, Mat_Final_B, 0, 0, 0, current_input,
                0, 0, 0, 0, temp_B, 0, 0);

    real_2d_array next_state;
    next_state.setlength(n_lifting, 1);

    for (int i = 0; i < 6; i++)
      q_simulation[i] = q_edmd[i];
    CalculateCtrModel();
    for (int i = 0; i < 3; i++)
      tip_position[i] = shape_sim[shape_sim.size() - 1][i];

    double x, y, z;

    x = shape_sim[shape_sim.size() - 1][0];
    y = shape_sim[shape_sim.size() - 1][1];
    z = shape_sim[shape_sim.size() - 1][2];

    std::vector<double> temp;
    temp.push_back(x);
    temp.push_back(y);
    temp.push_back(z);

    temp.push_back(x * x);
    temp.push_back(y * y);
    temp.push_back(z * z);

    temp.push_back(x * x * y);
    temp.push_back(x * x * z);
    temp.push_back(y * y * x);
    temp.push_back(y * y * z);
    temp.push_back(z * z * x);
    temp.push_back(z * z * y);

    EDMD_COUNT = 0;
    psi_All.erase(psi_All.end());
    psi_All.insert(psi_All.begin(), temp);
    UpdateRviz();

    // VERIFY EDMD RESULTS AT THE END OF THE LEARNING PHASE
    if (time_progress >= total_time_trajectory && !already_verified) {
      already_verified = true;
      //  VerifyEDMD();
    }
  }
}

// --------------------------------------------------------------------------------------
// VERIFY THE LEARNING
// --------------------------------------------------------------------------------------
void CtrController::VerifyEDMD() {

  real_2d_array temp_inp;
  temp_inp.setlength(n_inputs, 1);
  temp_inp[0][0] = q_edmd[0];
  temp_inp[1][0] = q_edmd[1];
  temp_inp[2][0] = q_edmd[2];
  temp_inp[3][0] = q_edmd[3];
  temp_inp[4][0] = q_edmd[4];
  temp_inp[5][0] = q_edmd[5];

  real_2d_array inp;
  inp.setlength(n_inputs, 1);

  for (int i = 0; i < 10; i++) {
    std::vector<double> gen_q(6);
    std::random_device rd;
    std::mt19937 gen(rd());
    double degr = 5 * M_PI / 180;
    for (int j = 0; j < 3; j++) {
      std::uniform_real_distribution<> dis(-0.002, 0.002);
      gen_q[j] = dis(gen);
    }
    for (int j = 3; j < 6; j++) {
      std::uniform_real_distribution<> dis(-degr, degr);
      gen_q[j] = dis(gen);
    }
    for (int j = 0; j < 6; j++)
      inp[j][0] = temp_inp[j][0] + gen_q[j];
    CompareResults(inp);
  }
  CalculateCtrModel();
}

// --------------------------------------------------------------------------------------
// COMPARE PREDICTED RESULTS WITH MODEL
// --------------------------------------------------------------------------------------
void CtrController::CompareResults(real_2d_array inp) {
  real_2d_array temp_A;
  temp_A.setlength(n_lifting, 1);
  real_2d_array temp_B;
  temp_B.setlength(n_lifting, 1);
  real_2d_array current_state;
  current_state.setlength(n_lifting, 1);

  for (int i = 0; i < n_lifting; i++)
    current_state[i][0] = psi_All[0][i];
  real_2d_array current_input;
  current_input.setlength(n_inputs, 1);
  for (int i = 0; i < n_inputs; i++)
    current_input[i][0] = inp[i][0];
  real_2d_array next_state;
  next_state.setlength(n_lifting, 1);

  rmatrixgemm(n_lifting, 1, n_lifting, 1, Mat_Final_A, 0, 0, 0, current_state,
              0, 0, 0, 0, temp_A, 0, 0);
  rmatrixgemm(n_lifting, 1, n_inputs, 1, Mat_Final_B, 0, 0, 0, current_input, 0,
              0, 0, 0, temp_B, 0, 0);

  for (int i = 0; i < n_lifting; i++)
    next_state[i][0] = temp_A[i][0] + temp_B[i][0];

  q_edmd[0] = current_input[0][0];
  q_edmd[1] = current_input[1][0];
  q_edmd[2] = current_input[2][0];
  q_edmd[3] = current_input[3][0];
  q_edmd[4] = current_input[4][0];
  q_edmd[5] = current_input[5][0];
  for (int i = 0; i < 6; i++)
    q_simulation[i] = q_edmd[i];

  std::cout << "input vals " << q_simulation[0] << " " << q_simulation[1] << " "
            << q_simulation[2] << " " << q_simulation[3] << " "
            << q_simulation[4] << " " << q_simulation[5] << "\n";
  CalculateCtrModel();
  std::cout << "EDMD predicted res " << next_state[0][0] << " "
            << next_state[1][0] << " " << next_state[2][0] << "\n";

  double error =
      sqrt((shape_sim[shape_sim.size() - 1][0] - next_state[0][0]) *
               (shape_sim[shape_sim.size() - 1][0] - next_state[0][0]) +
           (shape_sim[shape_sim.size() - 1][1] - next_state[1][0]) *
               (shape_sim[shape_sim.size() - 1][1] - next_state[1][0]) +
           (shape_sim[shape_sim.size() - 1][2] - next_state[2][0]) *
               (shape_sim[shape_sim.size() - 1][2] - next_state[2][0]));
  std::cout << "ERROR is " << error << "\n";
  std::cout << std::flush;
}

// --------------------------------------------------------------------------------------
// CALCULATE NEXT JOINT INPUTS ACCORDING TO MPC RESULTS
// --------------------------------------------------------------------------------------
void CtrController::MPCStepping() {

  const int STATES = 12;
  const int CONTROLS = 6;

  matrix<double, STATES, STATES> A;
  for (int i = 0; i < STATES; i++) {
    for (int j = 0; j < STATES; j++) {
      A(i, j) = Mat_Final_A(i, j);
    }
  }

  matrix<double, STATES, CONTROLS> B;
  for (int i = 0; i < STATES; i++) {
    for (int j = 0; j < CONTROLS; j++) {
      B(i, j) = Mat_Final_B(i, j);
    }
  }

  matrix<double, STATES, 1> C;

  const int HORIZON = 5;
  matrix<double, STATES, 1> Q;
  for (int i = 0; i < STATES; i++) {
    C(i, 0) = 0;
    Q(i, 0) = 0;
  }
  Q(0, 0) = MPC_Q;
  Q(1, 0) = MPC_Q;
  Q(2, 0) = MPC_Q;
  matrix<double, CONTROLS, 1> R, lower, upper;

  R(0, 0) = MPC_R_TR;
  R(1, 0) = MPC_R_TR;
  R(2, 0) = MPC_R_TR;

  R(3, 0) = MPC_R_ROT;
  R(4, 0) = MPC_R_ROT;
  R(5, 0) = MPC_R_ROT;

  double tr_limit = MAX_STEP_TRANSLATIONAL;
  double rot_limit = MAX_STEP_ROTATIONAL;

  tr_limit = MAX_STEP_TRANSLATIONAL;
  rot_limit = MAX_STEP_ROTATIONAL;

  lower(0, 0) = q_edmd[0] - tr_limit;
  lower(1, 0) = q_edmd[1] - tr_limit;
  lower(2, 0) = q_edmd[2] - tr_limit;

  lower(3, 0) = q_edmd[3] - rot_limit;
  lower(4, 0) = q_edmd[4] - rot_limit;
  lower(5, 0) = q_edmd[5] - rot_limit;

  upper(0, 0) = q_edmd[0] + tr_limit;
  upper(1, 0) = q_edmd[1] + tr_limit;
  upper(2, 0) = q_edmd[2] + tr_limit;

  upper(3, 0) = q_edmd[3] + rot_limit;
  upper(4, 0) = q_edmd[4] + rot_limit;
  upper(5, 0) = q_edmd[5] + rot_limit;

  // MAKE SURE TUBES CANNOT PASS EACH OTHER
  double offset = 0.01;

  for (int i = 0; i < 3; i++) {
    if (lower(i, 0) < hard_llimit[i]) {
      lower(i, 0) = hard_llimit[i];
      upper(i, 0) = hard_llimit[i] + 2 * tr_limit;
    }
  }

  if (upper(2, 0) > 0) {
    upper(2, 0) = 0;
    lower(2, 0) = upper(2, 0) - 2 * tr_limit;
  }

  if (lower(0, 0) > lower(1, 0)) {
    lower(1, 0) = lower(0, 0) + offset;
    upper(1, 0) = lower(1, 0) + 2 * tr_limit;
  }
  if (lower(1, 0) > lower(2, 0)) {
    lower(2, 0) = lower(1, 0) + offset;
    upper(2, 0) = lower(2, 0) + 2 * tr_limit;
  }

  mpc<STATES, CONTROLS, HORIZON> controller(A, B, C, Q, R, lower, upper);
  controller.set_epsilon(0.1);
  controller.set_max_iterations(1000);
  matrix<double, STATES, 1> current_state;

  double x, y, z;
  x = tip_disturbed[0];
  y = tip_disturbed[1];
  z = tip_disturbed[2];

  std::vector<double> temp;

  temp.push_back(x);
  temp.push_back(y);
  temp.push_back(z);

  temp.push_back(x * x);
  temp.push_back(y * y);
  temp.push_back(z * z);

  temp.push_back(x * x * y);
  temp.push_back(x * x * z);
  temp.push_back(y * y * x);
  temp.push_back(y * y * z);
  temp.push_back(z * z * x);
  temp.push_back(z * z * y);
  for (int i = 0; i < STATES; i++) {
    current_state(i) = temp[i];
  }

  matrix<double, STATES, 1> target;

  for (int i = 0; i < HORIZON; i++) {
    for (int j = 0; j < 3; j++) {
      if (total_time_trajectory >= (time_progress + (dt * (i))))
        target(j, 0) = a_c[j][0] * (time_progress + (dt * (i))) + a_c[j][1];
      else
        target(j, 0) = a_c[j][0] * total_time_trajectory + a_c[j][1];
    }
    for (int j = 3; j < STATES; j++)
      target(j, 0) = 0;
    controller.set_target(target, i);
  }

  for (int i = 0; i < 3; i++)
    desired_pos[i] = target(i, 0);

  UpdateRviz();

  matrix<double, CONTROLS, 1> action;
  matrix<double, STATES, 1> current_state_next;

  action = controller(current_state);

  for (int i = 0; i < 6; i++)
    if (action(i, 0) == 0) {
      action(i, 0) = q_edmd[i];
    }

  q_edmd[0] = action(0, 0);
  q_edmd[1] = action(1, 0);
  q_edmd[2] = action(2, 0);

  q_edmd[3] = action(3, 0);
  q_edmd[4] = action(4, 0);
  q_edmd[5] = action(5, 0);

  for (int i = 0; i < 6; i++)
    q_simulation[i] = q_edmd[i];
  CalculateCtrModel();

  time_progress += dt;

  stepping_file << q_simulation[0] << "," << q_simulation[1] << ","
                << q_simulation[3] << "," << q_simulation[4] << ",,"
                << target(0, 0) << "," << target(1, 0) << "," << target(2, 0)
                << ",," << shape_sim[shape_sim.size() - 1][0] << ","
                << shape_sim[shape_sim.size() - 1][1] << ","
                << shape_sim[shape_sim.size() - 1][2] << ",,"
                << current_state_next(0, 0) << "," << current_state_next(1, 0)
                << "," << current_state_next(2, 0) << "\n";
}

// --------------------------------------------------------------------------------------
// GENERATE TRAJECTORY BASED ON VELOCITY AND SAMPLE TIME
// --------------------------------------------------------------------------------------
double CtrController::GenerateTrajectoryWithConstantSpeed(double vel,
                                                          double sample_time) {
  double dist[3] = {0, 0, 0};
  real_2d_array A("[[0,1],[1,1]]");
  real_1d_array b = "[0,0]";
  ae_int_t n = 2;
  ae_int_t info;

  current_pos[0] = prev_end_pos[0];
  current_pos[1] = prev_end_pos[1];
  current_pos[2] = prev_end_pos[2];

  double max = 0;
  double distance =
      sqrt((current_pos[0] - end_pos[0]) * (current_pos[0] - end_pos[0]) +
           (current_pos[1] - end_pos[1]) * (current_pos[1] - end_pos[1]) +
           (current_pos[2] - end_pos[2]) * (current_pos[2] - end_pos[2]));

  target_pos[0] = end_pos[0];
  target_pos[1] = end_pos[1];
  target_pos[2] = end_pos[2];

  max = distance / vel;
  for (int i = 0; i < 3; i++) {
    a_c[i][1] = current_pos[i];
    dist[i] = abs(current_pos[i] - end_pos[i]);
    if (current_pos[i] > end_pos[i])
      a_c[i][0] = -dist[i] / max;
    else
      a_c[i][0] = dist[i] / max;
  }
  return distance / vel;
}

} // namespace ctr_controller