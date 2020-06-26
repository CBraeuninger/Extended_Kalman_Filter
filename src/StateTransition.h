#ifndef STATETRANSITION_H_
#define STATETRANSITION_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class StateTransition : public MatrixXd{
    public:
        /* Constructor and destructor */
        StateTransition();
        virtual ~StateTransition();

        MatrixXd matrix_; // state transition matrix

        void update(long long deltaT);

        StateTransition transpose();

        MatrixXd operator + (MatrixXd other){
            return other + matrix_;
        };
};

#endif // STATETRANSITION_H_