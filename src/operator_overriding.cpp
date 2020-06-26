
#include "Eigen/Dense"
#include "StateTransition.h"
#include "JacobianH.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

// overloaded functions for (non-commutative) multiplying:

MatrixXd multiply(JacobianH Hj, MatrixXd other){
    return Hj.matrix_  * other;
};

MatrixXd multiply(MatrixXd other, JacobianH Hj){
    return other * Hj.matrix_;
}

MatrixXd multiply(MatrixXd M, StateTransition st){
    return M * st.matrix_; 
}

MatrixXd multiply(StateTransition st, MatrixXd M){
    return st.matrix_ * M;
}

VectorXd multiply(StateTransition st, VectorXd vec){
    std::cout<<"matrix: "<<st.matrix_<<"vector: "<<vec<<std::endl;
    return st.matrix_ * vec;
}

StateTransition multiply(StateTransition st1, StateTransition st2){
    StateTransition result;
    result.matrix_ = st1.matrix_ * st2.matrix_;
    return result;
}