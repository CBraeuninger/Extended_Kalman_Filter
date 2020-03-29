#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"
#include "ProcessNoise.h"
#include "StateTransition.h"
#include "JacobianH.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // process noise parameters
  int sigma_ax_;
  int sigma_ay_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  // Matrices for Kalman filter equations
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_;
  JacobianH Hj_;
  ProcessNoise Q_;
  StateTransition F_;
  Eigen::MatrixXd P_;

};

#endif // FusionEKF_H_
