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

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // Set the state mapping function for lidar
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Set the state transition matrix
  // This will get updated later with timestep values for the velocity entries in the first and second rows
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  // set the acceleration noise components to be used by the process noise matrix Q later
  float noise_ax = 5;
  float noise_ay = 5;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) 
  {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      // Get the data from the raw measurements
      float range = measurement_pack.raw_measurements_[0];
      float bearing = measurement_pack.raw_measurements_[1];
      float range_rate = measurement_pack.raw_measurements_[2];

      float px = range*sin(bearing);
      float py  = range*cos(bearing);
      // Iffy on whether this is the correct way to get velocities from range rate
      float vx = range_rate*sin(bearing);
      float vy  = range_rate*cos(bearing); 

      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
      ekf_.x_(2) = vx;
      ekf_.x_(3) = vy;

      // Update the previous message timestamp
      previous_timestamp_ = measurement_pack.timestamp_;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      // TODO: Initialize state.
      // Get the position values from the raw measurement
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];

      ekf_.x_(0) = px;
      ekf_.x_(1) = py;

      // Update the previous message timestamp
      previous_timestamp_ = measurement_pack.timestamp_;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
