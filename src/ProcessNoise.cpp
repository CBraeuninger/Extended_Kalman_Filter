#include "ProcessNoise.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;

ProcessNoise::ProcessNoise(){

    // process noise parameters
    sigma_ax_ = 9;
    sigma_ay_ = 9;
    
    matrix_ = MatrixXd(4,4);
    matrix_ <<   0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
}

ProcessNoise::~ProcessNoise(){
}

void ProcessNoise::update(double deltaT){
      matrix_ << 0.25 * pow(sigma_ax_, 2) * pow(deltaT, 4), 0, 0.5 * pow(sigma_ax_, 2) * pow(deltaT, 3), 0,
        0, 0.25 * pow(sigma_ay_, 2) * pow(deltaT, 2), 0, 0.5 * pow(sigma_ay_, 2) * pow(deltaT, 3),
        0.5 * pow(sigma_ax_, 2) * pow(deltaT,3), 0, pow(sigma_ax_, 2) * pow(deltaT, 2), 0,
        0, 0.5 * pow(sigma_ay_,2) * pow(deltaT, 3), 0, pow(sigma_ay_,2) * pow(deltaT, 2);
}