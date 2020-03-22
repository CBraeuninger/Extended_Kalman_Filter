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

  // process noise parameters
  int sigma_ax_ = 9;
  int sigma_ay_ = 9;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2); //measurement covariance matrix (laser) : describing uncertainity of sensor measurement
  R_radar_ = MatrixXd(3, 3); //measurement covariance matrix (radar): describing uncertainity of sensor measurement
  H_laser_ = MatrixXd(2, 4); // H projects our belief of the object's current state into the measurement state of the sensor
  Hj_ = MatrixXd(3, 4); // for radar we need the Jacobian of the measurement function, since it is non-linear
  
  Q_ = MatrixXd(4,4); // process noise

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Lidar measures only the position (px, py) of the object
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Hj will be initialized with first sensor measurement

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.

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



void FusionEKF::calculateProcessNoiseQ(long long deltaT){

  Q_ << 0.25 * pow(sigma_ax_, 2) * pow(deltaT, 4), 0, 0.5 * pow(sigma_ax_, 2) * pow(deltaT, 3), 0,
        0, 0.25 * pow(sigma_ay_, 2) * pow(deltaT, 2), 0, 0.5 * pow(sigma_ay_, 2) * pow(deltaT, 3),
        0.5 * pow(sigma_ax_, 2) * pow(deltaT,3), 0, pow(sigma_ax_, 2) * pow(deltaT, 2), 0,
        0, 0.5 * pow(sigma_ay_,2) * pow(deltaT, 3), 0, pow(sigma_ay_,2) * pow(deltaT, 2);

}
