#ifndef VARIABLE_CURVATURE_ROBOT_SEGMENT_H
#define VARIABLE_CURVATURE_ROBOT_SEGMENT_H

#include <memory>
#include "../../lib/eigen/Eigen/Core"
//#include "../lib/eigen/Eigen/Core"
//#include "Eigen/Core"

class Tube {
public:
    double k;
    double phi;
    double l;
    Eigen::Matrix4d absTransform;

    Eigen::Matrix4d tubeLocalDiscreteFrame(double lPercentile);
    void setAbsTransform(Eigen::Matrix4d T) {absTransform = T;}
    Eigen::Matrix4d tubeLocalEndFrame(){
        return tubeLocalDiscreteFrame(1.0);
    }
    Eigen::MatrixXd jacobian();

    Eigen::Matrix4d fk(std::array<double,3> conf){
        auto [k_in, phi_in, l_in] = conf;
        this->k = k_in;
        this->phi = phi_in;
        this->l = l_in;
        return tubeLocalEndFrame();
    }

};


#endif //VARIABLE_CURVATURE_ROBOT_SEGMENT_H
