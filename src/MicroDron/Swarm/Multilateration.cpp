//
// Created by abiel on 9/16/20.
//

#include "Multilateration.h"
#include <Eigen/Dense>
#include <iostream>

Pose3D Multilateration::solveMultilateration(const std::vector<ReferenceMeasurement> &measurments) {
    if(measurments.size() < 3){
        throw std::runtime_error("More references are needed");
    }

    Eigen::MatrixXd A(measurments.size(), 4);

    Eigen::MatrixXd B(measurments.size(), 1);

    for(Eigen::Index i = 0; const auto &meas : measurments){
        const auto &pos(meas.referencePosition);
        const auto &dist(meas.distance);

        //A matrix assignment
        A(i, 0) = 1;
        A(i, 1) = -2.0 * pos.x;
        A(i, 2) = -2.0 * pos.y;
        A(i, 3) = -2.0 * pos.z;

        //B matrix assignment
        B(i) = std::pow(dist, 2.0) - std::pow(pos.x, 2.0) - std::pow(pos.y, 2.0) - std::pow(pos.z, 2.0);

        i++;
    }

    Eigen::MatrixXd x = A.colPivHouseholderQr().solve(B);
    //std::cout << x << std::endl; std::cout << std::endl;

    return {x(1), x(2), x(3)};
}