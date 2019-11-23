#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  num_states_to_estimate = 4;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
          0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
          0, 0.0009, 0,
          0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
          0, 1, 0, 0;
  Q_ = Eigen::MatrixXd(4, 4);
  F_= Eigen::MatrixXd(4, 4);
  F_.setIdentity();
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF()
{}


void FusionEKF::UpdateF(double dt)
{
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}

void FusionEKF::UpdateQ(double dt, double noise_ax, double noise_ay)
{

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
          0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
          dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
          0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{


  //Uncomment one of the lines if you don't want to use one of the meaurment type
  //if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) return;
  //if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) return;

  if (!is_initialized_)
  {

    previous_timestamp_ = measurement_pack.timestamp_;

    // first measurement
    cout << "EKF: " << endl;
    VectorXd x = VectorXd(num_states_to_estimate);
    x.setOnes();
    ekf_.Init(num_states_to_estimate);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      double angle = measurement_pack.raw_measurements_[1];
      double dist = measurement_pack.raw_measurements_[0];
      x(0) = sin(angle) * dist;
      x(1) = cos(angle) * dist;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      x(0) = measurement_pack.raw_measurements_[0];
      x(1) = measurement_pack.raw_measurements_[1];
    }
    ekf_.SetX(x);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  double elapsed_time = (previous_timestamp_ - measurement_pack.timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;


  Eigen::VectorXd z;

  UpdateF(elapsed_time);
  UpdateQ(elapsed_time, 9, 9);
  ekf_.Predict(F_, Q_);

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    z = Eigen::VectorXd(3);
    z[0] = measurement_pack.raw_measurements_[0];
    z[1] = measurement_pack.raw_measurements_[1];
    z[2] = measurement_pack.raw_measurements_[2];
    if(tools.CalculateJacobian(ekf_.GetX(), Hj_))
    {
      ekf_.UpdateEKF(z, Hj_, R_radar_);
    }
    else{
      //skip the measurment if calculating the Jacobian is not possible
      return;
    }
  }
  else
  {
    z = Eigen::VectorXd(2);
    z[0] = measurement_pack.raw_measurements_[0];
    z[1] = measurement_pack.raw_measurements_[1];
    ekf_.Update(z,H_laser_, R_laser_);
  }


  // print the output
  cout << "x_ = " << ekf_.GetX() << endl;
  cout << "P_ = " << ekf_.GetP() << endl;
}
