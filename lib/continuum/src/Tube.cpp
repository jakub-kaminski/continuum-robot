#include "Tube.h"

Eigen::Matrix4d Tube::tubeLocalDiscreteFrame(double lPercentile){
    double l = this->l * lPercentile;
    Eigen::Matrix4d localT;
    //@formatter:off
    localT << cos(phi)*cos(k*l), -sin(phi), cos(phi)*sin(k*l), (cos(phi)*(1-cos(k*l)))/k,
            sin(phi)*cos(k*l), cos(phi),  sin(phi)*sin(k*l), (sin(phi)*(1-cos(k*l)))/k,
            -sin(k*l),         0.0,       cos(k*l),          (sin(k*l))/k,
            0.0,               0.0,       0.0,                1.0;
    //@formatter:on

    return localT;
}

Eigen::MatrixXd Tube::jacobian() {
    Eigen::MatrixXd res(6,3);
    //@formatter:off
    res <<
    cos(phi)*(cos(k*l) - 1)/pow(k,2), 0.0, 0.0,
    sin(phi)*(cos(k*l) - 1)/pow(k,2), 0.0, 0.0,
    -(sin(k*l)-k*l)/pow(k,2),         0.0, 1.0,
    -l*sin(phi),                      0.0, -k*sin(phi),
    l*cos(phi),                       0.0, k*cos(phi),
    0.0,                              1.0, 0.0;
    //@formatter:on
    return res;
}
