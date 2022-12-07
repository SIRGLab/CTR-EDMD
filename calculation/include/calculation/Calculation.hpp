#pragma once
#include "alglib/optimization.h"
#include "alglib/stdafx.h"
#include "custom_msg/calculation_acAction.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <algorithm>
#include <boost/numeric/odeint.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <vector>
using namespace alglib;
using namespace std;
using namespace boost::numeric::odeint;
using namespace std::chrono;
using namespace std;

namespace calculation {
#define PI 3.141592653589793
#define TUBES_N 3

class Calculation {
  actionlib::SimpleActionServer<custom_msg::calculation_acAction> as_;
  typedef std::vector<double> state_type;
  runge_kutta4<state_type> stepper_t;

public:
  static state_type y;
  static vector<double> result_pos[3];
  static vector<double> rot[9];
  static double q_0[2 * TUBES_N];
  static double q[2 * TUBES_N];
  static double d_tip[3];
  static double l[TUBES_N];
  static double l_k[TUBES_N];
  static double base_point;
  static double alpha[TUBES_N];
  static double alpha_prev[TUBES_N];
  static double alpha0;
  static double f[3];
  static double dist_f[3];
  static int method;

  static double a_c[3][3];
  static double u_init[5];
  static double E[TUBES_N];
  static double G[TUBES_N];
  static double UX[TUBES_N];

  static double q_0_initial[2 * TUBES_N];
  static double q_initial[2 * TUBES_N];
  static double uFinal[5];
  static double uInp[5];
  static double alphaFinal[3];
  static double yFinal[123];

  Calculation(ros::NodeHandle &nodeHandle);
  virtual ~Calculation();

  int BaseMax(double q_0[]);
  bool IsEqual(double a, double b, double epsilon);
  void Segmentation(double q[], double ss[], int &ss_n, double s[], int &s_n,
                    double *e, double *g, double L[], double *Ux, double UX[]);
  void SetDefaultValues(int s_n);
  void CalculateConstants(int s_n);
  std::tuple<double, double> CalculateCurvature(double alpha[3]);
  static void UpdateResults(const state_type y, const double t);
  void Ode(state_type &dydt, state_type &y, double t);
  void CalculateCtrShape(const real_1d_array &x0, real_1d_array &fi, void *ptr);
  bool MinimizeShape();
  void executeCB(const custom_msg::calculation_acGoalConstPtr &goal);
  bool CtrShape();

private:
};

} // namespace calculation
