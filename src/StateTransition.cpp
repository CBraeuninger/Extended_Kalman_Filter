#include "StateTransition.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

StateTransition::StateTransition(){
    F_ = MatrixXd(4,4);
    F_ <<   1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
}

void StateTransition::update(long long deltaT){

  F_ << 1, 0, deltaT, 0,
        0, 1, 0, deltaT,
        0, 0, 1, 0,
        0, 0, 0, 1;

}