#include "ProcessNoise.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ProcessNoise::ProcessNoise(){

    // process noise parameters
    sigma_ax_ = 9;
    sigma_ay_ = 9;
    
    Q_ = MatrixXd(4,4);
    Q_ <<   0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
}

void ProcessNoise::update(long long deltaT){
      Q_ << 0.25 * pow(sigma_ax_, 2) * pow(deltaT, 4), 0, 0.5 * pow(sigma_ax_, 2) * pow(deltaT, 3), 0,
        0, 0.25 * pow(sigma_ay_, 2) * pow(deltaT, 2), 0, 0.5 * pow(sigma_ay_, 2) * pow(deltaT, 3),
        0.5 * pow(sigma_ax_, 2) * pow(deltaT,3), 0, pow(sigma_ax_, 2) * pow(deltaT, 2), 0,
        0, 0.5 * pow(sigma_ay_,2) * pow(deltaT, 3), 0, pow(sigma_ay_,2) * pow(deltaT, 2);
}