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
      float range = measurement_pack.raw_measurements_(0);
      float bearing = measurement_pack.raw_measurements_(1);
      float range_rate = measurement_pack.raw_measurements_(2);

      float px = range*cos(bearing);
      float py  = range*sin(bearing);
      // Iffy on whether this is the correct way to get velocities from range rate
      float vx = range_rate*cos(bearing);
      float vy  = range_rate*sin(bearing); 

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
      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);

      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;

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

  // Get the elapsed time using the current timestamp and the previous one
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Define various powers of dt to make the process noise matrix cleaner
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // set the acceleration noise components to be used by the process noise matrix Q
  float noise_ax = 9;
  float noise_ay = 9;

  // Update the state transtion matrix with the delta time
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Define the process noise matrix Q with the delta time and noise values
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;


  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
    // TODO: Radar updates
    // Update state mapping and measurement noise matricies to the RADAR ones
    ekf_.R_ = R_radar_;

    // Calculate the updated Jacobian based on the current state
    Hj_ = tools.CalculateJacobian(ekf_.x_);

    ekf_.H_ = Hj_;

    // Pass the measurement into the update function
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } 
  else 
  {
    // TODO: Laser updates
    // Update state mapping and measurement noise matricies to the LiDAR ones
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    // Pass the measurement into the update function
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
