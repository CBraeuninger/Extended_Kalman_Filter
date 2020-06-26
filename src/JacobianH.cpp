#include "JacobianH.h"
#include "Eigen/Dense"
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

JacobianH::JacobianH(){

   matrix_ = MatrixXd(3,4);

}

JacobianH::~JacobianH(){
   
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

JacobianH JacobianH::transpose(){

   JacobianH Hj_transposed;
   Hj_transposed.matrix_ = matrix_.transpose();

   return Hj_transposed;
}

