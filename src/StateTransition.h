#ifndef STATETRANSITION_H_
#define STATETRANSITION_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;

class StateTransition : public MatrixXd{
    public:
        /* Constructor and destructor */
        StateTransition();
        virtual ~StateTransition();

        MatrixXd matrix_; // state transition matrix

        void update(long long deltaT);

        MatrixXd operator * (MatrixXd other){
            return matrix_ * other;
        };

        MatrixXd transpose (){
            return matrix_.transpose();
        };

};

#endif // STATETRANSITION_H_