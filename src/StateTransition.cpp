#include "StateTransition.h"

void StateTransition::update(long long deltaT){

  F_ << 1, 0, deltaT, 0,
        0, 1, 0, deltaT,
        0, 0, 1, 0,
        0, 0, 0, 1;

}