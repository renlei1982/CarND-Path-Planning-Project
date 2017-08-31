#ifndef TRAJECTORY_GEN_H_
#define TRAJECTORY_GEN_H_


#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "spline.h"

using namespace std;


constexpr double pi() { return M_PI; }



class T_GEN {

  public:

  /**
  * Constructor
  */
  T_GEN();

  /**
  * Destructor
  */
  virtual ~T_GEN();

  vector<vector<double>> Solve(double ref_v, int lane_num, double x, double y, double s, double d, double yaw,
                               double speed, vector<double> prev_path_x, vector<double> prev_path_y, double end_s, double end_d,
                               vector<double> mp_s, vector<double> mp_x, vector<double> mp_y);

};

#endif /*TRAJECTORY_GEN_H_*/
