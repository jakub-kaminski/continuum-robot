#pragma once

#include <vector>
#include <array>
#include <iostream>

#include "Tube.h"
#include "ContinuumUtils.h"
#include "../../lib/eigen/Eigen/Core"
#include "../../lib/eigen/unsupported/Eigen/MatrixFunctions"

class ContinuumRobotInterface {
public:
    int numTubes;
    std::vector<Tube> tubes;
    Eigen::Matrix4d robotBase = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d adjust;
    std::vector<std::array<Eigen::Matrix4d, 50>> discreteFrames;

    Eigen::MatrixXd robotJacobian();

    Eigen::MatrixXd effectorT;
    std::vector<std::array<double, 3>> config;

    Eigen::Matrix4d fk(std::vector<std::array<double, 3>> config);

    std::vector<std::array<double, 3>> ik(Eigen::Matrix4d pose);

    Eigen::MatrixXd getEffectorWorld(){
        return robotBase * effectorT;
    }

    std::vector<std::array<double, 3>> ikWorld(Eigen::Matrix4d pose){
        return ik(robotBase.inverse() * pose);
    }

    void updateDiscreteFrames();

    ContinuumRobotInterface(int numTubes) : numTubes(numTubes) {
        adjust << 1.0,  0.0, 0.0, 0.0,
                0.0,  0.0, 1.0, 0.0,
                0.0, -1.0, 0.0, 0.0,
                0.0,  0.0, 0.0, 1.0;

//        robotBase =  robotBase * ContinuumUtils::rotZ(M_PI_2) * ContinuumUtils::rotX(M_PI_2);
        robotBase = robotBase * ContinuumUtils::rotX(-M_PI_2) * ContinuumUtils::rotZ(M_PI) * ContinuumUtils::rotY(-M_PI_2);

        tubes.reserve(numTubes);
        discreteFrames.reserve(numTubes);

        std::array<Eigen::Matrix4d, 50> arr;
        for (int j = 0; j < 50; ++j) {
            arr[j] = Eigen::Matrix4d::Identity();
        }

        for (int i = 0; i < numTubes; ++i) {
            auto t = Tube{};
            tubes.push_back(t);

            discreteFrames.push_back(arr);
        }
    }

    std::vector<double> getVectorFromMatrix(Eigen::Matrix4d mat);

    std::vector<double> getTubeSegmentTransform(int tubeID, int segmentID);
};