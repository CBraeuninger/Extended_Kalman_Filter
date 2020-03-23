#ifndef STATETRANSITION_H_
#define STATETRANSITION_H_

#include "Eigen/Dense"

class StateTransition{
    public:
        /* Constructor and destructor */
        StateTransition();
        virtual ~StateTransition();

        Eigen::MatrixXd matrix_; // state transition matrix

        void update(long long deltaT);

};

#endif // STATETRANSITION_H_