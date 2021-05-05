#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "JacobianH.h"
#include "StateTransition.h"
#include "ProcessNoise.h"
#include "MeasurementPackage.h"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param Hj_ Jacobian of measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(const MeasurementPackage &measurements, Eigen::MatrixXd &P_in, StateTransition &F_in,
            Eigen::MatrixXd &H_in, JacobianH &Hj_in, Eigen::MatrixXd &R_radar_in,
            Eigen::MatrixXd &R_laser_in, ProcessNoise &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param deltaT Time between k and k+1 in s
   */
  void Predict(double deltaT);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  Eigen::VectorXd calculate_h_of_x();

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  StateTransition F_;

  // process covariance matrix
  ProcessNoise Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // Jacobian of measurment matrix
  JacobianH Hj_;

  // measurement covariance matrix (radar)
  Eigen::MatrixXd R_radar_;

  // measurement covariance matrix (laser)
  Eigen::MatrixXd R_laser_;

};

#endif // KALMAN_FILTER_H_
