#include "StateTransition.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;

StateTransition::StateTransition(){
    matrix_ = MatrixXd(4,4);
    matrix_ <<   1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
}

void StateTransition::update(long long deltaT){

  matrix_ << 1, 0, deltaT, 0,
        0, 1, 0, deltaT,
        0, 0, 1, 0,
        0, 0, 0, 1;

}