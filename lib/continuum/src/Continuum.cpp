#include "../include/Continuum/Continuum.h"
#include <chrono>
#include <unistd.h>


Eigen::Vector3d skew2vec(Eigen::Matrix3d m){
    Eigen::Vector3d vec;
    vec << m(2,1), m(0,2), m(1,0);
    return vec;
}

//Eigen::MatrixXd ContinuumRobotInterface::robotJacobian() {
//    Eigen::MatrixXd jacobian(6, tubes.size()*3);
////    jacobian.block(0,0,6,3) = tubes[0].jacobian();
//
//    // T will be accumulating the absolute transformation
//    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//    for(int i = 0; i < tubes.size(); ++i){
//        // local transformation matrix at base of the tube
//        // affects how contributions of each tube parameters
//        // affect the motion in the task space
//        Eigen::Matrix4d Tii;
//        if(i == 0) {
//            // the first tube uses robot base as a reference
//            Tii = robotBase;
//        }
//        else {
//            // each subsequent tube uses the end frame of the previous tube
//            // to define the coordinates for jacobian influence
//            Tii = tubes[i - 1].tubeLocalEndFrame();
//        }
//        T *= Tii;
//
//        Eigen::MatrixXd Ad(6,6);
//
////        std::cout << Ad <<std::endl;
//
//        Ad.block(0,0,3,3) = T.block(0,0,3,3);
//        Ad.block(0,3,3,3) = ContinuumUtils::skew(T(0,3), T(1,3), T(2,3)) * T.block(0,0,3,3);
//        Ad.block(3,0,3,3) = Eigen::Array33d::Zero();
//        Ad.block(3,3,3,3) = T.block(0,0,3,3);
//
////        std::cout << Ad <<std::endl;
////        auto test = tubes[i].jacobian();
////        std::cout << test << std::endl;
//
//        jacobian.block(0,i*3,6,3) = Ad * tubes[i].jacobian();
//    }
//    return jacobian;
//}

Eigen::MatrixXd ContinuumRobotInterface::robotJacobian() {
    Eigen::MatrixXd jacobian(6, tubes.size()*3);
    jacobian.block(0,0,6,3) = tubes[0].jacobian();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for(int i = 1; i < tubes.size(); ++i){
        Eigen::Matrix4d Tii = tubes[i-1].tubeLocalDiscreteFrame(1.0);
        T *= Tii;

        Eigen::MatrixXd Ad(6,6);

//        std::cout << Ad <<std::endl;

        Ad.block(0,0,3,3) = T.block(0,0,3,3);
        Ad.block(0,3,3,3) = ContinuumUtils::skew(T(0,3), T(1,3), T(2,3)) * T.block(0,0,3,3);
        Ad.block(3,0,3,3) = Eigen::Array33d::Zero();
        Ad.block(3,3,3,3) = T.block(0,0,3,3);

//        std::cout << Ad <<std::endl;
//        auto test = tubes[i].jacobian();
//        std::cout << test << std::endl;

        jacobian.block(0,i*3,6,3) = Ad * tubes[i].jacobian();
    }
    return jacobian;
}

Eigen::Matrix4d ContinuumRobotInterface::fk(std::vector<std::array<double, 3>> config) {
    this->config = config;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for(int i = 0; i < tubes.size(); ++i){
//        tubes[i].setAbsTransform(T);
        Eigen::Matrix4d Tnow = tubes[i].fk(config[i]);
        tubes[i].setAbsTransform(T);
        T *= Tnow;
    }
    effectorT = T;
    return T;
}

std::vector<std::array<double, 3>> ContinuumRobotInterface::ik(Eigen::Matrix4d targetT) {
    Eigen::Vector3d pTarget = targetT.block(0,3,3,1);

//    auto start = std::chrono::steady_clock::now();

    int iter = 0;
    while(true){
        Eigen::Matrix4d Tnow = fk(this->config);
        Eigen::Vector3d pCurrent = Tnow.block(0,3,3,1);
        Eigen::Matrix4d twistNowSkewMat4;
        twistNowSkewMat4 = (Tnow.inverse() * targetT).log();
        Eigen::Vector3d twistRotVec = skew2vec(twistNowSkewMat4.block(0,0,3,3));

        double linErr = (pTarget - pCurrent).norm();
        double rotErr = twistRotVec.norm();

        if(linErr < 0.001 && rotErr < 0.001){
//            auto end = std::chrono::steady_clock::now();
//            std::cout << "Iterations done: " << iter << ".   ";
//            std::cout << "Elapsed time in microseconds: "
//            << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " µs" << std::endl;

            return this->config;
        }
        ++iter;
        Eigen::MatrixXd J = robotJacobian();
        Eigen::MatrixXd Jpinv = ContinuumUtils::pseudoInverse(J);

        Eigen::MatrixXd twist(6,1);
        twist.block(0,0,3,1) = twistNowSkewMat4.block(0,3,3,1);
        twist.block(3,0,3,1) = twistRotVec;
        Eigen::VectorXd deltaQ = 0.1 * Jpinv * twist;

        double totalLen = config[0][2] + deltaQ[2] + config[1][2] + deltaQ[5] + config[2][2] + deltaQ[8];

        for(int i = 0; i < 3; ++i){
            config[i][0] += deltaQ(i*3);
            config[i][1] += deltaQ(i*3 + 1);
//            config[i][2] += deltaQ(i*3 + 2);
            config[i][2] = totalLen/3.0;
        }
    }
    return {{}};
}

void ContinuumRobotInterface::updateDiscreteFrames() {
    int segmentsPerTube = discreteFrames[0].size();
   for(int i = 0; i < numTubes; ++i){
       for(int j = 0; j < segmentsPerTube; ++j){
           double percentile = double(j + 1) / double(segmentsPerTube);
           discreteFrames[i][j] = robotBase * tubes[i].absTransform * tubes[i].tubeLocalDiscreteFrame(percentile);
       }
   }
}

std::vector<double> ContinuumRobotInterface::getVectorFromMatrix(Eigen::Matrix4d mat){
    std::vector<double> res;
    res.reserve(16);

    Eigen::Matrix4d matAdjusted = adjust * mat;

    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            res.push_back(matAdjusted(r, c)*1.0);
        }
    }
    return res;
}

std::vector<double> ContinuumRobotInterface::getTubeSegmentTransform(int tubeID, int segmentID) {
    return getVectorFromMatrix(discreteFrames[tubeID][segmentID]);
//    if(partNumber == 0) return getVectorFromMatrix(robots[id].body);
//    else return getVectorFromMatrix(robots[id].top);
}
