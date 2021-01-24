#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */

  // Predict the new state vector
  x_ = F_ * x_;

  // Predict the new State covariance matrix
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  // Calculate the measurement error
  VectorXd y = z - (H_*x_);

  MatrixXd S = H_ * P_* H_.transpose() + R_;

  // Calculate Kalman gain
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // Calculate the updated state and covariance matrix
  x_ = x_ + (K * y);

  // Define and identity matrix for the state
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  P_ = (I - K * H_)*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // Get the individual state variables from the state vector
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // Convert to polar
  float range = sqrt(px*px + py*py);
  float bearing = atan2(py, px);
  float range_rate;

  float thresh = 0.0001;

  // Make sure there is no divide by 0 error when range is very small
  if(range > thresh)
  {
    range_rate = (px*vx + py*vy)/range;
  }
  else
  {
    // There might be something better to do besides setting range rate to 0
    range_rate = 0;
  }
  
  VectorXd h_x;
  h_x = VectorXd(3);

  // Load the values into the array
  h_x << range, bearing, range_rate;

  // Calculate the measurement error
  VectorXd y = z - h_x;

  // Normalize the bearing angle
  while(y(1) > M_PI)
  {
    y(1) -= 2* M_PI;
  }
  while(y(1) < -M_PI)
  {
    y(1) += 2* M_PI;
  }

  MatrixXd S = H_ * P_* H_.transpose() + R_;

  // Calculate Kalman gain
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // Calculate the updated state and covariance matrix
  x_ = x_ + (K * y);

  // Define and identity matrix for the state
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  P_ = (I - K * H_)*P_;


}
