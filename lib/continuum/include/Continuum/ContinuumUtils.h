#pragma once

//#include "../lib/eigen/Eigen/Core"
//#include "Eigen/Core"

#include "../../lib/eigen/Eigen/Core"

namespace ContinuumUtils
{
    Eigen::Matrix4d rotX(double ang);
    Eigen::Matrix4d rotY(double ang);
    Eigen::Matrix4d rotZ(double ang);
//    Eigen::Matrix4d rotateFrameAroundFrameZ(Eigen::MatrixXd frame, Eigen::Matrix4d baseFrame, double angle);

    Eigen::Vector4d spiralPoint(double angle, double radius, double pitch);
    Eigen::MatrixXd spiralCompute();

    Eigen::Matrix3d skew(double x, double y, double z);

//    Eigen::Vector3d skew2vec(Eigen::Matrix3d m){
//     Eigen::Vector3d vec;
//        vec << m(2,1), m(0,2), m(1,0);
//        return vec;
//    }

    struct VelData{
        double pos;
        double minError;
        double vel;
        double lastTime;
        VelData(double pos, double minError, double vel, double lastTime) : pos(pos), minError(minError), vel(vel), lastTime(lastTime){}
    };

    // credits: https://gist.github.com/gokhansolak/d2abaefcf3e3b767f5bc7d81cfe0b36a
    template<typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }
}