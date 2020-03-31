#include "JacobianH.h"
#include "Eigen/Dense"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

JacobianH::JacobianH(){

   matrix_ = MatrixXd(4,4);

}

void JacobianH::update(const VectorXd& x_state){

   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);
 
   // Calculate rho
   float rho = std::sqrt(std::pow(px,2)+std::pow(py,2));

   if (rho==0)  // check division by zero => throws error
   {
      throw rho;
   }
   else // compute the Jacobian matrix
   {
      matrix_ << px / rho,                        py / rho,                        0,          0,
            -py / std::pow(rho,2),                px / std::pow(rho,2),                 0,          0,
            py * (vx*py-vy*px) / std::pow(rho,3), px * (vy*px-vx*py) / std::pow(rho,3), px / rho,   py / rho;

   }

}