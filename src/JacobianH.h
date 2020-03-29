#ifndef JACOBIANH_H_
#define JACOBIANH_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class JacobianH : public MatrixXd{

    public:

        JacobianH();
        ~JacobianH();

        MatrixXd matrix_;

        void JacobianH::update(const VectorXd& x_state);

        MatrixXd operator * (MatrixXd other){
            return matrix_  * other;
        };

};

#endif // JACOBIANH_H_