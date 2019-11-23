#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  int est_len = estimations.size();
  int variables_len = estimations[0].size();

  VectorXd res(variables_len);
  res.setZero();
  for (int i = 0; i<est_len; i++){
    for (int j = 0; j<variables_len;j++){
      double val = estimations[i][j] - ground_truth[i][j];
      res[j] += val*val;
    }
  }
  for (int j = 0; j<variables_len;j++)
  {
    res[j] = sqrt(res[j])/est_len;
  }

  /**
   * TODO: Calculate the RMSE here.
   */
   return res;
}

bool Tools::CalculateJacobian(const VectorXd& x_state, MatrixXd& Hj) {

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return false;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
          -(py/c1), (px/c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return true;
}
