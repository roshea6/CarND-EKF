#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  // 4 value vector to calculate the root mean square error of the state
  VectorXd rmse(4);
  // Fill it with 0s
  rmse << 0, 0, 0, 0;

  // Check to make sure inputs are valid
  if(estimations.size() == 0)
  {
    cout << "Error: received estimation vector of size 0" << endl;
    return rmse;
  }
  if(estimations.size() != ground_truth.size())
  {
    cout << "Error: received estimation vector with different size than ground truth vector" << endl;
    return rmse;
  }

  // Loop through through the estimations
  for(int i = 0; i < estimations.size(); i++)
  {
    // Find the residual (difference between estimation and groundtruth)
    VectorXd residual = estimations[i] - ground_truth[i];

    // Square the residual
    residual = residual.array() * residual.array();

    // Add the residual to the rmse
    rmse += residual;
  }

  // Divide by the number of measurements
  rmse /= estimations.size();

  // Calculate the square root
  rmse = rmse.array().sqrt();

  return rmse;

  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

  // Define the Jacobian Matrix as a 3x4
  MatrixXd Hj;

  // Fill the array with zeros to start
  Hj = MatrixXd::Zero(3, 4);

  // Get the state variables
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Ensure there won't be a divide by 0 error
  if((px == 0) and(py == 0))
  {
    cout << "Divide by 0 error in Jacobian calculation" << endl;
    return Hj;
  }

  float c1 = pow(px, 2) + pow(py, 2);
  float c2 = sqrt(c1);
  float c3 = pow(c1, 3/2);

  // Compute the Jacobian matrix
  Hj << px/c2, py/c2, 0, 0,
        -py/c1, px/c1, 0, 0,
        (py*(vx*py - vy*px))/c3, (px*(vy*px - vx*py))/c3, px/c2, py/c2;

  return Hj;
}
