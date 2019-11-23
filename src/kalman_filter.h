#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter
{
  public:
    /**
     * Constructor
     */
    KalmanFilter() {};

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    void Init(int num_states);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void Predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

    /**
 * Updates the state by using Extended Kalman Filter equations
 * @param z The measurement at k+1
 */
    void UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

    const Eigen::VectorXd &GetX() const
    { return x_; };

    const Eigen::MatrixXd &GetP() const
    { return P_; };

    void SetX(const Eigen::VectorXd &x) {x_ = x;};

  private:
    void Update(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);
    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;



};

#endif // KALMAN_FILTER_H_
