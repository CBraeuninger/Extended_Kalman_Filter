#include "StateTransition.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;

StateTransition::StateTransition(){
    matrix_ = MatrixXd(4,4);
    matrix_ <<   1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
}

StateTransition::~StateTransition(){
        
}

void StateTransition::update(double deltaT){

  matrix_ << 1, 0, deltaT, 0,
        0, 1, 0, deltaT,
        0, 0, 1, 0,
        0, 0, 0, 1;

}

StateTransition StateTransition::transpose(){

   StateTransition st_transposed;
   st_transposed.matrix_ = matrix_.transpose();

   return st_transposed;
}

