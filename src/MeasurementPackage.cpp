#include "MeasurementPackage.h"
#include "Eigen/Dense"

using Eigen::VectorXd;

MeasurementPackage::MeasurementPackage(){

    sensor_type_ = LASER;
    timestamp_ = 0;
    raw_measurements_ = VectorXd(2);

}

MeasurementPackage::~MeasurementPackage(){
}