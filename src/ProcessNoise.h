#ifndef PROCESSNOISE_H_
#define PROCESSNOISE_H_

#include "Eigen/Dense"

class ProcessNoise{

    public:

        /* Constructor and deconstructor */
        ProcessNoise();
        ~ProcessNoise();

        int sigma_ax_;
        int sigma_ay_;
        Eigen::VectorXd Q_;

        void ProcessNoise::update(long long deltaT);

};

#endif // PROCESSNOISE_H_