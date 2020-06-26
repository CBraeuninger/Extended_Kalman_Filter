#ifndef JACOBIANH_H_
#define JACOBIANH_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class JacobianH : public MatrixXd{

    public:

        JacobianH();
        virtual ~JacobianH();

        MatrixXd matrix_;

        void update(const VectorXd& x_state);

        JacobianH transpose();

};

#endif // JACOBIANH_H_