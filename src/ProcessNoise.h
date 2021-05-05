#ifndef PROCESSNOISE_H_
#define PROCESSNOISE_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;

class ProcessNoise{

    public:

        /* Constructor and deconstructor */
        ProcessNoise();
        virtual ~ProcessNoise();

        int sigma_ax_;
        int sigma_ay_;
        MatrixXd matrix_;

        void update(double deltaT);

        MatrixXd operator + (MatrixXd other){
            return matrix_ + other;
        };     

};

#endif // PROCESSNOISE_H_