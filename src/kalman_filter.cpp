#include "kalman_filter.h"

//TODO Remove
#include "iostream"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::~KalmanFilter()
{}


void KalmanFilter::Init(int num_states)
{
  x_ = VectorXd(num_states);
  x_.setZero();

  P_ = MatrixXd(num_states, num_states);
  P_.setIdentity() * 1000;
}

void KalmanFilter::Predict(const MatrixXd &F, const MatrixXd &Q)
{
  x_ = F * x_;
  MatrixXd Ft = F.transpose();
  P_ = F * P_ * Ft + Q;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R)
{
  VectorXd z_pred = VectorXd(3);
  z_pred[0] = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
  z_pred[1] = atan2(x_[1], x_[0]);
  if (z_pred[0] < 0.0001)
  {
    z_pred[2] = (x_[0] * x_[2] + x_[1] * x_[3]) / 0.0001;
  }
  else
  {
    z_pred[2] = ((x_[0] * x_[2] + x_[1] * x_[3])) / z_pred[0];
  }
  VectorXd y = z - z_pred;
  while (y[1] > 3.14)
  {
    y[1] = y[1] - 6.28;
  }
  while (y[1] < -3.14)
  {
    y[1] = y[1] + 6.28;
  }
  UpdateCommon(y, H, R);

}

void KalmanFilter::UpdateCommon(const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R)
{

  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd(K.rows(), H.cols());
  I.setIdentity();
  P_ = (I - K * H) * P_;
}


void KalmanFilter::Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R)
{

  VectorXd z_pred = H * x_;
  VectorXd y = z - z_pred;
  UpdateCommon(y, H, R);


}

