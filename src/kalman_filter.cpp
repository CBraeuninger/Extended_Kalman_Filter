#include "kalman_filter.h"
#include <math.h>
#include "StateTransition.h"
#include "ProcessNoise.h"
#include "JacobianH.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, StateTransition &F_in,
                        MatrixXd &H_in, JacobianH &Hj_in, MatrixXd &R_radar_in,
                        MatrixXd &R_laser_in, ProcessNoise &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_radar_ = R_radar_in;
  R_laser_ = R_laser_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict(long long deltaT) {
  /**
   * predict the state
   */
  // update state transition matrix
  F_.update(deltaT);
  // update the process noise covariance matrix
  Q_.update(deltaT);
  // Update x (in Cartesian coordinates)
  x_ = F_ * x_;
  // Calculate new state covariance matrix
  P_ = Q_ + F_ * P_ * F_.transpose();
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_laser_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
  // calculate state in polar coordinates
  VectorXd y = z - calculate_h_of_x();
  MatrixXd S = Hj_ * P_ * Hj_.transpose() + R_radar_;
  MatrixXd K = P_ * Hj_.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * Hj_) * P_;
}

VectorXd KalmanFilter::calculate_h_of_x(){

  VectorXd h_ = VectorXd(3);
  h_ << sqrt(pow(x_(0),2)+pow(x_(1),2)), atan2(x_(1), x_(0)), (x_(0)*x_(2)+x_(1)*x_(3))/sqrt(pow(x_(0),2)+pow(x_(1),2));
  return h_;
}
