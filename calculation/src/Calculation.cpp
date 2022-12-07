#include "calculation/Calculation.hpp"
#include <actionlib/server/simple_action_server.h>
namespace calculation {
double Calculation::q_0[2 * TUBES_N] = {-0.28, -0.14, -0.0945, 0, 0, 0};
double Calculation::q[2 * TUBES_N] = {0.0, 0.0, 0, 0, 0, 0};
double Calculation::q_0_initial[2 * TUBES_N] = {-0.28, -0.14, -0.0945, 0, 0, 0};
double Calculation::q_initial[2 * TUBES_N] = {0.0, 0.0, 0, 0, 0, 0};
vector<double> Calculation::result_pos[3];
vector<double> Calculation::rot[9];
typedef std::vector<double> state_type;
state_type Calculation::y(123);
double Calculation::d_tip[3];
double Calculation::alpha[TUBES_N];
double Calculation::alpha0;
double Calculation::f[3] = {0, 0, 0};
double Calculation::dist_f[3] = {0, 0, 0};
int Calculation::method = 2;
double Calculation::a_c[3][3];
double Calculation::u_init[5];
double Calculation::uFinal[5];
double Calculation::uInp[5];
double Calculation::alphaFinal[3];

// --------------------------------------------------------------------------------------
// SET PHYSICAL PARAMETERS OF THE TUBES
// --------------------------------------------------------------------------------------
double d_outer[TUBES_N] = {2 * 0.55e-3, 2 * 0.9e-3, 2 * 1.1e-3};
double d_inner[TUBES_N] = {2 * 0.35e-3, 2 * 0.7e-3, 2e-3};
double Calculation::E[TUBES_N] = {6.43e+10, 5.25e+10, 4.71e+10};

double J[TUBES_N] = {1.0e-11 * 0.0120, 1.0e-11 * 0.0653, 1.0e-11 * 0.1686};
double I[TUBES_N] = {1.0e-12 * 0.0601, 1.0e-12 * 0.3267, 1.0e-12 * 0.8432};
double Calculation::G[TUBES_N] = {2.5e+10, 2.14e+10, 2.97e+10};
double Calculation::UX[TUBES_N] = {14, 10, 3.5};
double UY[TUBES_N] = {0, 0, 0};
double EI[TUBES_N];
double GJ[TUBES_N];
double e[TUBES_N][TUBES_N * TUBES_N];
double g[TUBES_N][TUBES_N * TUBES_N];
double ei[TUBES_N * TUBES_N][TUBES_N];
double gj[TUBES_N * TUBES_N][TUBES_N];

double Calculation::yFinal[123];
int SEG_CNT = 0;
double initial_curvature[5];
double ode_time_step = 0.5;
vector<vector<double>> joint_pos;
ofstream error_file;

double Calculation::l[TUBES_N] = {0.5050, 0.251, 0.174};
double Calculation::l_k[TUBES_N] = {0.1454, 0.1471, 0.134};
double uz0[TUBES_N] = {0, 0, 0};

double GammaTemp[25];

double Ux[TUBES_N][TUBES_N * TUBES_N];
double Uy[TUBES_N][TUBES_N * TUBES_N];
double Uz[TUBES_N] = {0, 0, 0};

double alpha_new[TUBES_N];
vector<double> y_all[123];
vector<double> r[3];

vector<double> u1x;
vector<double> u1y;

vector<double> U_z[3];
vector<double> Length;
double R_0t[9];
int seg_cnt = 0;

// --------------------------------------------------------------------------------------
// INITIALIZE NODE, START ACTION
// --------------------------------------------------------------------------------------
Calculation::Calculation(ros::NodeHandle &nodeHandle)
    : as_(nodeHandle, "", boost::bind(&Calculation::executeCB, this, _1),
          false) {
  ROS_INFO("Calculation Node is active!");
  for (int i = 0; i < 3; i++) {
    J[i] = (M_PI * (pow(d_outer[i], 4) - pow(d_inner[i], 4))) / 32;
    I[i] = (M_PI * (pow(d_outer[i], 4) - pow(d_inner[i], 4))) / 64;
  }

  as_.start();
}

// --------------------------------------------------------------------------------------
// UPDATE INPUT VALUES FOR THE MODEL (JOINTS, PARAMETERS) -> SOLVE BVP
// --------------------------------------------------------------------------------------
void Calculation::executeCB(
    const custom_msg::calculation_acGoalConstPtr &goal) {
  custom_msg::calculation_acResult result_;
  for (int i = 0; i < 6; i++) {
    q[i] = 0;
    q_0[i] = goal->q[i];
  }
  for (int i = 0; i < 3; i++) {
    E[i] = goal->e[i];
    G[i] = goal->g[i];
  }
  ros::Rate r(1000);

  MinimizeShape();

  // --------------------------------------------------------------------------------------
  // SEND BACK SHAPE AND ORIANTAION
  // --------------------------------------------------------------------------------------
  int i = 0;
  while (i < result_pos->size()) {
    result_.x.push_back(result_pos[0][i]);
    result_.y.push_back(result_pos[1][i]);
    result_.z.push_back(result_pos[2][i]);

    result_.R11.push_back(rot[0][i]);
    result_.R12.push_back(rot[1][i]);
    result_.R13.push_back(rot[2][i]);

    result_.R21.push_back(rot[3][i]);
    result_.R22.push_back(rot[4][i]);
    result_.R23.push_back(rot[5][i]);

    result_.R31.push_back(rot[6][i]);
    result_.R32.push_back(rot[7][i]);
    result_.R33.push_back(rot[8][i]);

    i++;
  }
  as_.setSucceeded(result_);
}

Calculation::~Calculation() {}

// --------------------------------------------------------------------------------------
// SET DEFAULT VALUES
// --------------------------------------------------------------------------------------
void Calculation::SetDefaultValues(int s_n) {
  y.resize(0);
  y.shrink_to_fit();
  Length.resize(0);
  Length.shrink_to_fit();
  u1x.resize(0);
  u1x.shrink_to_fit();
  u1y.resize(0);
  u1y.shrink_to_fit();
  for (int i = 0; i < TUBES_N; i++) {
    EI[i] = 0;
    GJ[i] = 0;
    r[i].resize(0);
    r[i].shrink_to_fit();
    result_pos[i].resize(0);
    result_pos[i].shrink_to_fit();
    U_z[i].resize(0);
    U_z[i].shrink_to_fit();

    uz0[i] = 0;
  }
  for (int i = 0; i < 9; i++) {
    rot[i].resize(0);
    rot[i].shrink_to_fit();
  }
  for (int i = 0; i < s_n; i++) {
    ei[i][0] = 0;
    ei[i][1] = 0;
    ei[i][2] = 0;

    ei[0][i] = 0;
    ei[1][i] = 0;
    ei[2][i] = 0;

    Ux[0][i] = 0;
    Ux[1][i] = 0;
    Ux[2][i] = 0;

    Uy[0][i] = 0;
    Uy[1][i] = 0;
    Uy[2][i] = 0;

    R_0t[i] = 0;
  }
  for (int i = 0; i < 5; i++)
    uFinal[i] = 0;
  for (int i = 0; i < 123; i++) {
    y_all[i].resize(0);
    y_all[i].shrink_to_fit();
  }
}

// --------------------------------------------------------------------------------------
// UPDATE RESULTS DURING SOLVING ODE
// --------------------------------------------------------------------------------------
void Calculation::UpdateResults(const state_type y, const double t) {
  Length.push_back(t);

  r[0].push_back(y[2 * TUBES_N]);
  r[1].push_back(y[2 * TUBES_N + 1]);
  r[2].push_back(y[2 * TUBES_N + 2]);

  result_pos[0].push_back(y[2 * TUBES_N]);
  result_pos[1].push_back(y[2 * TUBES_N + 1]);
  result_pos[2].push_back(y[2 * TUBES_N + 2]);

  U_z[0].push_back(y[0]);
  U_z[1].push_back(y[1]);
  U_z[2].push_back(y[2]);

  u1x.push_back(y[18]);
  u1y.push_back(y[19]);

  alpha_new[0] = y[TUBES_N];
  alpha_new[1] = y[TUBES_N + 1];
  alpha_new[2] = y[TUBES_N + 2];
  alphaFinal[0] = y[TUBES_N];
  alphaFinal[1] = y[TUBES_N + 1];
  alphaFinal[2] = y[TUBES_N + 2];
  for (int i = 0; i < 9; i++) {
    R_0t[i] = y[9 + i];
    rot[i].push_back(y[9 + i]);
  }

  uFinal[0] = y[18];
  uFinal[1] = y[19];
  uFinal[2] = y[0];
  uFinal[3] = y[1];
  uFinal[4] = y[2];

  for (int i = 0; i < 60; i++)
    yFinal[i] = y[i];

  return;
}

// --------------------------------------------------------------------------------------
// FIND BASE OF CTR
// --------------------------------------------------------------------------------------
int Calculation::BaseMax(double q_0[]) {
  double max = 100;
  int max_id = 0;
  for (int i = 0; i < TUBES_N; i++) {
    if (q_0[i] < max) {
      max = q_0[i];
      max_id = i;
    }
  }
  return max_id;
}

// --------------------------------------------------------------------------------------
// HELP FUNCTION TO COMPARE VALUES
// --------------------------------------------------------------------------------------
bool Calculation::IsEqual(double a, double b, double epsilon) {
  return std::abs(a - b) < epsilon;
}

// --------------------------------------------------------------------------------------
// SEGMENT THE CTR
// --------------------------------------------------------------------------------------
void Calculation::Segmentation(double q[], double ss[], int &ss_n, double s[],
                               int &s_n, double *e, double *g, double L[],
                               double *Ux, double UX[]) {
  // --------------------------------------------------------------------------------------
  // GET THE REFERENCE TUBE'S VALUE
  // --------------------------------------------------------------------------------------
  int max_id = BaseMax(q_0);
  ss_n = 1;

  // --------------------------------------------------------------------------------------
  // CALCULATE THE TUBE SEGMENTS
  // ss[0] is the reference point. The segments are compared with this value
  // --------------------------------------------------------------------------------------
  ss[0] = -1 * (q_0[max_id] + q[max_id]);
  double reference = ss[0];
  for (int i = 0; i < TUBES_N; i++) {
    double temp = q_0[i] + q[i] + ss[0];
    if (i != max_id)
      ss[ss_n++] = q_0[i] + q[i] + ss[0]; // Start points of the tubes
    ss[ss_n++] = l[i] + temp;             // End points of the tubes
    ss[ss_n++] = l[i] + (temp - l_k[i]);  // End of straight parts of the tubes
    d_tip[i] = q_0[i] + l[i]; // Storing the tip of each tube -> these are the
                              // positions where one of the tubes ends
  }

  // --------------------------------------------------------------------------------------
  // SORTING THE ARRAY IN ASCENDING ORDER
  // --------------------------------------------------------------------------------------
  std::sort(ss, ss + ss_n);

  // --------------------------------------------------------------------------------------
  // CALCULATE s[] -> segments after fixed point
  // --------------------------------------------------------------------------------------
  s_n = 0;
  for (int i = 0; i < ss_n; i++)
    if (ss[i] + 1 * (q_0[max_id] + q[max_id]) > 0) {
      s[s_n++] = ss[i] + 1 * (q_0[max_id] + q[max_id]);
    }
  L[0] = ss[0];
  for (int i = 1; i < ss_n; i++)
    L[i] = ss[i] - ss[i - 1];

  // --------------------------------------------------------------------------------------
  // CALCULATE SEGMENT VALUES
  // --------------------------------------------------------------------------------------
  for (int i = 0; i < TUBES_N; i++) {
    for (int j = 0; j < s_n; j++) {
      double temp = l[i] + q_0[i] + q[i];
      // --------------------------------------------------------------------------------------
      // CALCULATE e[] -> it's E[i] if the segment contains the tube
      // --------------------------------------------------------------------------------------
      if (s[j] < temp || IsEqual(s[j], temp, 0.0001)) {
        *((e + i * TUBES_N * TUBES_N) + j) = E[i];
        *((g + i * TUBES_N * TUBES_N) + j) = G[i];
      } else {
        *((e + i * TUBES_N * TUBES_N) + j) = 0;
        *((g + i * TUBES_N * TUBES_N) + j) = 0;
      }

      // --------------------------------------------------------------------------------------
      // CALCULATE ux[] -> it's Ux[i] if the segment contains the curved part of
      // the tube
      // --------------------------------------------------------------------------------------
      if ((temp - l_k[i] < s[j]) && (!IsEqual(temp - l_k[i], s[j], 0.0001)) &&
          (s[j] < temp || IsEqual(s[j], temp, 0.0001)))
        *((Ux + i * TUBES_N * TUBES_N) + j) = UX[i];
      else
        *((Ux + i * TUBES_N * TUBES_N) + j) = 0;
    }
  }

  return;
}

// --------------------------------------------------------------------------------------
// ODE
// --------------------------------------------------------------------------------------
void Calculation::Ode(state_type &dydt, state_type &y, double t) {
  // --------------------------------------------------------------------------------------
  // CALCULATE EI
  // --------------------------------------------------------------------------------------
  double EI_sum = 0;
  for (int i = 0; i < TUBES_N; i++) {
    EI_sum += ei[SEG_CNT][i];
  }
  // --------------------------------------------------------------------------------------
  // Rtheta
  // --------------------------------------------------------------------------------------
  real_2d_array Rtheta[3];
  for (int i = 0; i < 3; i++) {
    Rtheta[i].setlength(3, 3);
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Rtheta[0][i][j] = 0;
      Rtheta[1][i][j] = 0;
      Rtheta[2][i][j] = 0;
    }
  }
  for (int i = 0; i < 3; i++) {
    Rtheta[i][0][0] = cos(y[TUBES_N + i]);
    Rtheta[i][0][1] = -sin(y[TUBES_N + i]);
    Rtheta[i][1][0] = sin(y[TUBES_N + i]);
    Rtheta[i][1][1] = cos(y[TUBES_N + i]);
    Rtheta[i][2][2] = 1;
  }
  // --------------------------------------------------------------------------------------
  // RTtheta
  // --------------------------------------------------------------------------------------
  real_2d_array RTtheta[3];
  for (int i = 0; i < 3; i++) {
    RTtheta[i].setlength(3, 3);
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      RTtheta[0][i][j] = 0;
      RTtheta[1][i][j] = 0;
      RTtheta[2][i][j] = 0;
    }
  }
  for (int i = 0; i < 3; i++) {
    RTtheta[i][0][0] = cos(y[TUBES_N + i]);
    RTtheta[i][0][1] = sin(y[TUBES_N + i]);
    RTtheta[i][1][0] = -sin(y[TUBES_N + i]);
    RTtheta[i][1][1] = cos(y[TUBES_N + i]);
    RTtheta[i][2][2] = 1;
  }
  // --------------------------------------------------------------------------------------
  // dRTtheta
  // --------------------------------------------------------------------------------------
  real_2d_array dRTtheta[3];
  for (int i = 0; i < 3; i++) {
    dRTtheta[i].setlength(3, 3);
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      dRTtheta[0][i][j] = 0;
      dRTtheta[1][i][j] = 0;
      dRTtheta[2][i][j] = 0;
    }
  }
  for (int i = 0; i < 3; i++) {
    dRTtheta[i][0][0] = -sin(y[TUBES_N + i]);
    dRTtheta[i][0][1] = cos(y[TUBES_N + i]);
    dRTtheta[i][1][0] = -cos(y[TUBES_N + i]);
    dRTtheta[i][1][1] = -sin(y[TUBES_N + i]);
  }
  // --------------------------------------------------------------------------------------
  // dRtheta
  // --------------------------------------------------------------------------------------
  real_2d_array dRtheta[3];
  for (int i = 0; i < 3; i++) {
    dRtheta[i].setlength(3, 3);
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      dRtheta[0][i][j] = 0;
      dRtheta[1][i][j] = 0;
      dRtheta[2][i][j] = 0;
    }
  }
  for (int i = 0; i < 3; i++) {
    dRtheta[i][0][0] = -sin(y[TUBES_N + i]);
    dRtheta[i][0][1] = -cos(y[TUBES_N + i]);
    dRtheta[i][1][0] = cos(y[TUBES_N + i]);
    dRtheta[i][1][1] = -sin(y[TUBES_N + i]);
  }

  // --------------------------------------------------------------------------------------
  // SET CURVATURES OF THE FIRST TUBE
  // --------------------------------------------------------------------------------------
  real_2d_array u[3];
  u[0].setlength(3, 1);
  u[1].setlength(3, 1);
  u[2].setlength(3, 1);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      u[i][j][0] = 0;
    }
  }

  u[0][0][0] = y[18]; // u1x
  u[0][1][0] = y[19]; // u1y
  u[0][2][0] = y[0];  // u1z

  // --------------------------------------------------------------------------------------
  // CALCULATE CURVATURES
  // ui = (R^T_i*R'_i)^v = R^T_theta*u1 + theta'*e3
  // --------------------------------------------------------------------------------------
  for (int i = 1; i < 3; i++) {
    rmatrixgemm(3, 1, 3, 1, RTtheta[i], 0, 0, 0, u[0], 0, 0, 0, 0, u[i], 0, 0);
    u[i][2][0] = y[i]; // - y[0];//?
  }

  // --------------------------------------------------------------------------------------
  // u_hat
  // --------------------------------------------------------------------------------------
  real_2d_array u_hat[3];
  for (int k = 0; k < 3; k++) {
    u_hat[k].setlength(3, 3);
    double u_hat_temp[TUBES_N][TUBES_N] = {{0, -u[k][2][0], u[k][1][0]},
                                           {u[k][2][0], 0, -u[k][0][0]},
                                           {-u[k][1][0], u[k][0][0], 0}};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        u_hat[k][i][j] = u_hat_temp[i][j];
      }
    }
  }

  // --------------------------------------------------------------------------------------
  // SET INITIAL VALUES
  // --------------------------------------------------------------------------------------
  for (int i = 0; i < 20; i++) {
    dydt[i] = 0;
  }

  real_2d_array K[3];
  for (int i = 0; i < 3; i++) {
    K[i].setlength(3, 3);
    double K_temp[3][3] = {ei[SEG_CNT][i], 0, 0, 0, ei[SEG_CNT][i], 0, 0, 0,
                           gj[SEG_CNT][i]};
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        K[i][j][k] = K_temp[j][k];
      }
    }
  }

  // --------------------------------------------------------------------------------------
  // Calculate duiz
  // duiz = (ei[i]/gj[i])*(uix*Uiy-uiy*Uix)
  // dthetai = uiz-u1z
  // --------------------------------------------------------------------------------------
  double *pUx = (double *)Ux;
  double *pUy = (double *)Uy;

  for (int i = 0; i < TUBES_N; i++) {
    if (gj[SEG_CNT][i] != 0) {
      dydt[i] = u[i][0][0] * Uy[i][SEG_CNT] - u[i][1][0] * Ux[i][SEG_CNT];
      dydt[i] *= (ei[SEG_CNT][i] / gj[SEG_CNT][i]);
      dydt[TUBES_N + i] = y[i] - y[0];
    }
  }

  // --------------------------------------------------------------------------------------
  // T1 = dTheta_i * K_i * dRTtheta -> T1_hat
  // --------------------------------------------------------------------------------------
  real_2d_array T1[3];
  for (int i = 0; i < 3; i++) {
    T1[i].setlength(3, 1);
    real_2d_array T1_temp;
    T1_temp.setlength(3, 3);
    rmatrixgemm(3, 3, 3, 1, K[i], 0, 0, 0, dRTtheta[i], 0, 0, 0, 0, T1_temp, 0,
                0);
    rmatrixgemm(3, 1, 3, dydt[3 + i], T1_temp, 0, 0, 0, u[0], 0, 0, 0, 0, T1[i],
                0, 0);
  }

  // --------------------------------------------------------------------------------------
  // T2 = K_i * (u_i - U) -> T2_hat
  // --------------------------------------------------------------------------------------
  real_2d_array T2[3];
  real_2d_array T2_hat[3];
  for (int i = 0; i < 3; i++) {
    real_2d_array T2_temp2;
    T2_temp2.setlength(3, 1);
    T2_temp2[0][0] = u[i][0][0] - Ux[i][SEG_CNT];
    T2_temp2[1][0] = u[i][1][0] - Uy[i][SEG_CNT];
    T2_temp2[2][0] = u[i][2][0];

    T2[i].setlength(3, 1);
    rmatrixgemm(3, 1, 3, 1, K[i], 0, 0, 0, T2_temp2, 0, 0, 0, 0, T2[i], 0, 0);

    T2_hat[i].setlength(3, 3);
    double temp_t2_hat[3][3] = {{0, -T2[i][2][0], T2[i][1][0]},
                                {T2[i][2][0], 0, -T2[i][0][0]},
                                {-T2[i][1][0], T2[i][0][0], 0}};
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        T2_hat[i][j][k] = temp_t2_hat[j][k];
      }
    }
  }

  // --------------------------------------------------------------------------------------
  // T
  // --------------------------------------------------------------------------------------
  real_2d_array T[3];
  for (int i = 0; i < 3; i++) {
    T[i].setlength(3, 1);
    real_2d_array Temp1;
    Temp1.setlength(3, 1);
    rmatrixgemm(3, 1, 3, 1, u_hat[i], 0, 0, 0, T2[i], 0, 0, 0, 0, Temp1, 0, 0);
    for (int j = 0; j < 3; j++) {
      T[i][j][0] = T1[i][j][0] + Temp1[j][0];
    }
  }

  double e3[3] = {0, 0, 1};

  // --------------------------------------------------------------------------------------
  // Define R
  // --------------------------------------------------------------------------------------
  double R[TUBES_N][TUBES_N] = {
      {y[2 * TUBES_N + 3], y[2 * TUBES_N + 4], y[2 * TUBES_N + 5]},
      {y[2 * TUBES_N + 6], y[2 * TUBES_N + 7], y[2 * TUBES_N + 8]},
      {y[2 * TUBES_N + 9], y[2 * TUBES_N + 10], y[2 * TUBES_N + 11]}};

  real_2d_array u1_sum;
  u1_sum.setlength(3, 1);
  u1_sum[0][0] = 0;
  u1_sum[1][0] = 0;
  u1_sum[2][0] = 0;
  real_2d_array u1_temp[3];
  for (int i = 0; i < 3; i++) {
    u1_temp[i].setlength(3, 1);
    rmatrixgemm(3, 1, 3, 1, Rtheta[i], 0, 0, 0, T[i], 0, 0, 0, 0, u1_temp[i], 0,
                0);
  }
  for (int i = 0; i < 3; i++) {
    u1_sum[0][0] += u1_temp[i][0][0];
    u1_sum[1][0] += u1_temp[i][1][0];
    u1_sum[2][0] += u1_temp[i][2][0];
  }
  dydt[18] = -u1_sum[0][0] / EI_sum;
  dydt[19] = -u1_sum[1][0] / EI_sum;

  real_2d_array R_mat;
  R_mat.setlength(3, 3);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R_mat[i][j] = R[i][j];
    }
  }

  // --------------------------------------------------------------------------------------
  // dr=R*e3
  // --------------------------------------------------------------------------------------
  real_2d_array dr;
  dr.setlength(3, 1);
  real_2d_array e3_mat;
  e3_mat.setlength(3, 1);
  e3_mat[0][0] = 0;
  e3_mat[1][0] = 0;
  e3_mat[2][0] = 1;
  rmatrixgemm(3, 1, 3, 1, R_mat, 0, 0, 0, e3_mat, 0, 0, 0, 0, dr, 0, 0);
  for (int i = 0; i < TUBES_N; i++)
    dydt[2 * TUBES_N + i] = dr[i][0];

  // --------------------------------------------------------------------------------------
  // dR1=R1*u_hat;
  // --------------------------------------------------------------------------------------
  real_2d_array dR;
  dR.setlength(3, 3);
  rmatrixgemm(3, 3, 3, 1, R_mat, 0, 0, 0, u_hat[0], 0, 0, 0, 0, dR, 0, 0);
  for (int i = 0; i < TUBES_N; i++) {
    for (int j = 0; j < TUBES_N; j++) {
      dydt[9 + i * 3 + j] = dR[i][j];
    }
  }
}

// --------------------------------------------------------------------------------------
// CALCULATE CONSTANT VALUES
// --------------------------------------------------------------------------------------
void Calculation::CalculateConstants(int s_n) {
  for (int i = 0; i < TUBES_N; i++) {
    EI[i] = E[i] * I[i];
    GJ[i] = G[i] * J[i];
  }
  for (int i = 0; i < s_n; i++) {
    ei[i][0] = e[0][i] * I[0];
    ei[i][1] = e[1][i] * I[1];
    ei[i][2] = e[2][i] * I[2];

    gj[i][0] = g[0][i] * J[0];
    gj[i][1] = g[1][i] * J[1];
    gj[i][2] = g[2][i] * J[2];
  }
  return;
}

// --------------------------------------------------------------------------------------
// CALCULATE CURVATURE
// --------------------------------------------------------------------------------------
std::tuple<double, double> Calculation::CalculateCurvature(double alpha[3]) {
  double EI_sum = 0;
  double GJ_sum = 0;

  for (int i = 0; i < TUBES_N; i++) {
    EI_sum += ei[seg_cnt][i];
    GJ_sum += gj[seg_cnt][i];
  }
  double u1_x = 0;
  double u1_y = 0;

  for (int j = 0; j < TUBES_N; j++) {
    u1_x += ei[0][j] * Ux[j][0] * cos(-alpha[j]) +
            ei[0][j] * Uy[j][0] * sin(-alpha[j]);
    u1_y -= ei[0][j] * Ux[j][0] * sin(-alpha[j]) +
            ei[0][j] * Uy[j][0] * cos(-alpha[j]);
  }
  u1_x /= EI_sum;
  u1_y /= EI_sum;
  return std::make_tuple(u1_x, u1_y);
}

bool Calculation::CtrShape() {
  real_1d_array x0 = "[0,0,0,0,0]";
  real_1d_array f0 = "[0,0,0,0,0]";
  // ------------------------------------------------------------------------------------------
  // calculate initial curvature by solving the IVP
  // ------------------------------------------------------------------------------------------
  SetDefaultValues(20);

  double results[10][20];
  double B[TUBES_N] = {q[0] + q_0[0], q[1] + q_0[1], q[2] + q_0[2]};
  double ss[TUBES_N * TUBES_N];
  int ss_n = 0;
  double s[TUBES_N * TUBES_N];
  int s_n = 0;
  double L[TUBES_N * TUBES_N];

  fill(&Uy[0][0], &Uy[0][0] + sizeof(Uy), 0);

  Segmentation(q, ss, ss_n, s, s_n, (double *)e, (double *)g, L, (double *)Ux,
               UX);

  CalculateConstants(s_n);

  alpha0 = q_0[3] + q[3];

  for (int i = TUBES_N; i < TUBES_N + 3; i++) {
    alpha[i - TUBES_N] = q_0[i] + q[i] - alpha0;
  }

  tie(x0[0], x0[1]) = CalculateCurvature(alpha);
  CalculateCtrShape(x0, f0, NULL);
}
// --------------------------------------------------------------------------------------
// CALCULATE CTR SHAPE AND ORIANTAION
// --------------------------------------------------------------------------------------
void Calculation::CalculateCtrShape(const real_1d_array &x0, real_1d_array &fi,
                                    void *ptr) {
  // ------------------------------------------------------------------------------------------
  // Calculate tube parameters
  // ------------------------------------------------------------------------------------------
  SetDefaultValues(20);
  double results[10][20];
  double B[TUBES_N] = {q[0] + q_0[0], q[1] + q_0[1], q[2] + q_0[2]};
  double ss[TUBES_N * TUBES_N];
  int ss_n = 0;
  double s[TUBES_N * TUBES_N];
  int s_n = 0;
  double L[TUBES_N * TUBES_N];
  double u1_x, u1_y;
  fill(&Uy[0][0], &Uy[0][0] + sizeof(Uy), 0);

  Segmentation(q, ss, ss_n, s, s_n, (double *)e, (double *)g, L, (double *)Ux,
               UX);

  CalculateConstants(s_n);

  alpha0 = q_0[3] + q[3];

  for (int i = TUBES_N; i < TUBES_N + 3; i++) {
    alpha[i - TUBES_N] = q_0[i] + q[i] - alpha0;
  }

  //******************************************************************************************************
  // Set initial values
  //******************************************************************************************************
  for (int i = 0; i < 3; i++)
    uz0[i] = x0[i + 2];

  double r0[3] = {0, 0, 0};
  double R0[9] = {cos(alpha0),
                  -sin(alpha0),
                  0,
                  sin(alpha0),
                  cos(alpha0),
                  0,
                  0,
                  0,
                  1}; // 3x3
  state_type y_0(20);

  for (int i = 0; i < TUBES_N; i++) {
    y_0[i] = uz0[i];
    y_0[i + TUBES_N] = alpha[i];
    y_0[i + 2 * TUBES_N] = r0[i];
  }

  for (int i = 0; i < 9; i++)
    y_0[9 + i] = R0[i];

  u1_x = x0[0];
  u1_y = x0[1];

  //******************************************************************************************************
  // GOING THROUGH THE SEGMENTS
  //******************************************************************************************************
  double u[3][3];
  for (int seg_cnt = 0; seg_cnt < s_n; seg_cnt++) {
    SEG_CNT = seg_cnt;

    // ------------------------------------------------------------------------------------------
    // SET INPUT VALUES: y0
    // ------------------------------------------------------------------------------------------
    for (int i = 0; i < TUBES_N; i++) {
      y_0[i] = uz0[i];
      y_0[i + TUBES_N] = alpha[i];
      y_0[i + 2 * TUBES_N] = r0[i];
    }
    for (int i = 0; i < 9; i++) {
      y_0[9 + i] = R0[i];
    }
    y_0[18] = u1_x;
    y_0[19] = u1_y;

    // ------------------------------------------------------------------------------------------
    // Solving the ODE
    // ------------------------------------------------------------------------------------------
    if (SEG_CNT == 0)
      integrate_adaptive(
          stepper_t,
          [this](state_type y, state_type &dydt, double t) {
            this->Ode(dydt, y, t);
          },
          y_0, 0.0, s[SEG_CNT], 0.05,
          [this](state_type &y, double t) { this->UpdateResults(y, t); });

    else
      integrate_adaptive(
          stepper_t,
          [this](state_type y, state_type &dydt, double t) {
            this->Ode(dydt, y, t);
          },
          y_0, s[SEG_CNT - 1], s[SEG_CNT], 0.05,
          [this](state_type &y, double t) { this->UpdateResults(y, t); });

    // ------------------------------------------------------------------------------------------
    // Ensure continuity of the segments
    // ------------------------------------------------------------------------------------------
    alpha[0] = alpha_new[0];
    alpha[1] = alpha_new[1];
    alpha[2] = alpha_new[2];

    // ------------------------------------------------------------------------------------------
    // IF IT ISN'T THE LAST SEGMENT
    // ------------------------------------------------------------------------------------------
    if (SEG_CNT + 1 < s_n) {
      // ------------------------------------------------------------------------------------------
      // UPDATE VALUES ACCORDING TO THE RESULTS FROM THE LAST SEGMENT
      // ------------------------------------------------------------------------------------------
      uz0[0] = U_z[0].back();
      uz0[1] = U_z[1].back();
      uz0[2] = U_z[2].back();

      double dtheta[3];
      for (int k = 0; k < 3; k++) {
        dtheta[k] = uz0[k] - uz0[0];
      }
      r0[0] = r[0].back();
      r0[1] = r[1].back();
      r0[2] = r[2].back();

      for (int j = 0; j < 9; j++) {
        R0[j] = R_0t[j];
      }

      // ------------------------------------------------------------------------------------------
      // SET THE CURVATURES OF THE 3 TUBES
      // ------------------------------------------------------------------------------------------
      u[0][0] = u1x.back();
      u[0][1] = u1y.back();
      u[0][2] = alpha[0];
      for (int i = 0; i < 2; i++) {
        u[i + 1][0] = u[0][0] * cos(alpha[i + 1]) + u[0][1] * sin(alpha[i + 1]);
        u[i + 1][1] =
            -u[0][0] * sin(alpha[i + 1]) + u[0][1] * cos(alpha[i + 1]);
        u[i + 1][2] = dtheta[i + 1] - dtheta[0];
      }

      // ------------------------------------------------------------------------------------------
      // CALCULATE K INVERSE
      // ------------------------------------------------------------------------------------------
      double K_inv[3];
      K_inv[0] =
          1 / (ei[SEG_CNT + 1][0] + ei[SEG_CNT + 1][1] + ei[SEG_CNT + 1][2]);
      K_inv[1] = K_inv[0];
      K_inv[2] =
          1 / (gj[SEG_CNT + 1][0] + gj[SEG_CNT + 1][1] + gj[SEG_CNT + 1][2]);

      // ------------------------------------------------------------------------------------------
      // CALCULATE INITIAL CURVATURES FOR THE NEXT SEGMENT
      // ------------------------------------------------------------------------------------------
      double u1new[3] = {0, 0, 0};
      for (int j = 0; j < 3; j++) {
        double x_part = ei[SEG_CNT][j] * (u[j][0] - Ux[j][SEG_CNT]);
        double y_part = ei[SEG_CNT][j] * (u[j][1] - Uy[j][SEG_CNT]);
        u1new[0] += cos(alpha[j]) * x_part - sin(alpha[j]) * y_part;
        u1new[1] += sin(alpha[j]) * x_part + cos(alpha[j]) * y_part;

        x_part = ei[SEG_CNT + 1][j] * Ux[j][SEG_CNT + 1];
        y_part = ei[SEG_CNT + 1][j] * Uy[j][SEG_CNT + 1];
        u1new[0] += cos(alpha[j]) * x_part - sin(alpha[j]) * y_part;
        u1new[1] += sin(alpha[j]) * x_part + cos(alpha[j]) * y_part;

        u1new[2] -= gj[SEG_CNT + 1][j] * dtheta[j];
      }
      u1new[0] *= K_inv[0];
      u1new[1] *= K_inv[1];
      u1new[2] *= K_inv[2];
      u1_x = u1new[0];
      u1_y = u1new[1];
    }

    // ------------------------------------------------------------------------------------------
    // IF IT IS THE LAST SEGMENT
    // ------------------------------------------------------------------------------------------
    else {
      u[0][0] = yFinal[18];
      u[0][1] = yFinal[19];
      u[0][2] = yFinal[0];
    }
  }

  // ------------------------------------------------------------------------------------------
  // Calculate cost function
  // ------------------------------------------------------------------------------------------
  double Cost[5] = {u[0][0] - Ux[0][s_n - 1], u[0][1] - Uy[0][s_n - 1], u[0][2],
                    0, 0};
  double d_tip[3] = {
      l[0] + q[0] + q_0[0],
      l[1] + q[1] + q_0[1],
      l[2] + q[2] + q_0[2],
  };

  double min[3] = {1000, 1000, 1000};
  int min_id[3] = {0, 0, 0};

  for (int i = 1; i < 3; i++) {
    for (int j = 0; j < Length.size(); j++) {
      if (abs(Length[j] - d_tip[i] < min[i])) {
        min[i] = abs(Length[j] - d_tip[i]);
        min_id[i] = j;
      }
    }
  }
  Cost[3] = U_z[1][min_id[1]];
  Cost[4] = U_z[2][min_id[2]];

  fi[0] = Cost[0];
  fi[1] = Cost[1];
  fi[2] = Cost[2];
  fi[3] = Cost[3];
  fi[4] = Cost[4];
}

void ProxyFunction(const alglib::real_1d_array &x0, alglib::real_1d_array &f0,
                   void *ptr) {
  ((Calculation *)ptr)->CalculateCtrShape(x0, f0, &ptr);
}

bool Calculation::MinimizeShape() {
  real_1d_array x0 = "[0,0,0,0,0]";

  // ------------------------------------------------------------------------------------------
  // calculate initial curvature by solving the IVP
  // ------------------------------------------------------------------------------------------
  SetDefaultValues(20);

  double results[10][20];
  double B[TUBES_N] = {q[0] + q_0[0], q[1] + q_0[1], q[2] + q_0[2]};
  double ss[TUBES_N * TUBES_N];
  int ss_n = 0;
  double s[TUBES_N * TUBES_N];
  int s_n = 0;
  double L[TUBES_N * TUBES_N];

  fill(&Uy[0][0], &Uy[0][0] + sizeof(Uy), 0);

  Segmentation(q, ss, ss_n, s, s_n, (double *)e, (double *)g, L, (double *)Ux,
               UX);

  CalculateConstants(s_n);

  alpha0 = q_0[3] + q[3];

  for (int i = TUBES_N; i < TUBES_N + 3; i++) {
    alpha[i - TUBES_N] = q_0[i] + q[i] - alpha0;
  }

  tie(x0[0], x0[1]) = CalculateCurvature(alpha);

  // ------------------------------------------------------------------------------------------
  // MAKE SURE INITIAL CURVATURES ARE VALID
  // ------------------------------------------------------------------------------------------
  if (std::isnan(x0[0]) or std::isnan(x0[1]) or std::isnan(x0[2]) or
      std::isnan(x0[3]) or std::isnan(x0[4])) {
    ROS_INFO("INPUTS ARE NOT VALID\n");
    return false;
  }

  // ------------------------------------------------------------------------------------------
  // MINIMIZE
  // ------------------------------------------------------------------------------------------
  real_1d_array s0 = "[1,1,1,1,1]";
  double epsx = 0.00000001;
  ae_int_t maxits = 0;
  minlmstate state;
  minlmreport rep;

  minlmcreatev(5, x0, 0.0001, state); // 0.0001
  minlmsetcond(state, epsx, maxits);
  minlmsetscale(state, s0);

  real_1d_array f0 = "[0,0,0,0,0]";
  int b = 5;
  alglib::minlmoptimize(state, ProxyFunction, NULL, this);

  minlmresults(state, x0, rep);

  u_init[0] = x0[0];
  u_init[1] = x0[1];
  u_init[2] = x0[2];
  u_init[3] = x0[3];
  u_init[4] = x0[4];

  initial_curvature[0] = u_init[0];
  initial_curvature[1] = u_init[1];
  initial_curvature[2] = u_init[2];
  initial_curvature[3] = u_init[3];
  initial_curvature[4] = u_init[4];

  return true;
}

} // namespace calculation