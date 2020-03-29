#ifndef JACOBIANH_H_
#define JACOBIANH_H_

#include "Eigen/Dense"

class JacobianH : public Eigen::MatrixXd{

    public:

        JacobianH();
        ~JacobianH();

        Eigen::MatrixXd matrix_;

        void JacobianH::update(const VectorXd& x_state);

};

#endif // JACOBIANH_H_