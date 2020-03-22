#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculates the Jacobian of input state x_state (for radar measurement)
   */

    MatrixXd Hj(3,4);
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);
 
   // Calculate rho
   float rho = sqrt(pow(px,2)+pow(py,2));

   if (rho==0)  // check division by zero => throws error
   {
      throw rho;
   }
   else // compute the Jacobian matrix
   {
      Hj << px / rho,                        py / rho,                        0,          0,
            -py / pow(rho,2),                px / pow(rho,2),                 0,          0,
            py * (vx*py-vy*px) / pow(rho,3), px * (vy*px-vx*py) / pow(rho,3), px / rho,   py / rho;

   }
  
  return Hj;
}
