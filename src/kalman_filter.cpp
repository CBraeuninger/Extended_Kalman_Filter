#include "kalman_filter.h"
#include <math.h>
#include "StateTransition.h"
#include "ProcessNoise.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, StateTransition &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, ProcessNoise &Q_in) {
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
  // Update x (in Cartesian coordinates)
  x_ = F_ * x_;
  // calculate state in polar coordinates
  calculate_h_of_x();
  // Calculate new state covariance matrix
  P_ = Q_ + F_ * P_ * F_.transpose();
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}

void KalmanFilter::calculate_h_of_x(){

  h_ << sqrt(pow(x_(0),2)+pow(x_(1),2)), atan2(x_(1), x_(0)), (x_(0)*x_(2)+x_(1)*x_(3))/sqrt(pow(x_(0),2)+pow(x_(1),2));

}
