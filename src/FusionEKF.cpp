#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

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
  R_laser_ = MatrixXd(2, 2); //measurement covariance matrix (laser) : describing uncertainity of sensor measurement
  R_radar_ = MatrixXd(3, 3); //measurement covariance matrix (radar): describing uncertainity of sensor measurement
  H_laser_ = MatrixXd(2, 4); // H projects our belief of the object's current state into the measurement state of the sensor
  P_ = MatrixXd(4,4); // uncertainity of where we are
  Q_ = ProcessNoise();
  F_ = StateTransition();

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

  // uncertainity of where we are
  P_ << 1000, 0, 0, 0,
        0, 1000, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  // Hj will be initialized with first sensor measurement
  Hj_ = JacobianH();

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

    // Initialize the state ekf_.x_ with the first measurement
    
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // calculate Jacobian
      Hj_.update(ekf_.x_);
      // initialize Kalman filter
      ekf_.Init(ekf_.x_, P_, F_, Hj_, R_radar_, Q_);      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){
      ekf_.Init(ekf_.x_, P_, F_, H_laser_, R_laser_, Q_);
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
