#include <vector>
#include <iostream>
//#include "Eigen/Core"
#include "Continuum/Continuum.h"
#include "VariableCurvatureRobot.h"
#include "SpiralData.h"

#include <chrono>

int main(){
    VariableCurvatureRobot robot(3);
//    std::vector<std::array<double,3>> config = {
//            {0.001, 0.0, 1.0},
//            {0.05, 0.0, 1.0},
//            {0.2, 0.0, 4.0}
//    };
    std::vector<std::array<double,3>> config = {
        { 0.552649229, -83.24960826, 1.237827 },
        { 0.5579576148, 85.24228818, 1.308040 },
        { 0.102175075, -1.895120665, 2.699483 }
    };

    Eigen::Matrix4d beginPose = robot.fk(config);
    std::cout << "begin Pose:" << std::endl;
    std::cout << beginPose << std::endl;

    Eigen::Matrix4d goalPose = Eigen::Matrix4d::Identity();
    for(auto& el : pathpoints){
    goalPose(0,3) = el[0];
    goalPose(1,3) = el[1];
    goalPose(2,3) = el[2];

    std::vector<std::array<double, 3>> configRes = robot.ik(goalPose);
    Eigen::Matrix4d computedPose = robot.fk(configRes);

    std::cout << "L1: " << configRes[0][2] << " L2: " << configRes[1][2] << " L3: " << configRes[2][2] << std::endl;
    for(auto el : configRes){
        std::cout << "{ ";
        for(auto a : el) std::cout << a << ", ";
        std::cout << " }," << std::endl;
    }

    std::cout << " *************** Goal Pose: ***************" << std::endl;
//    std::cout << goalPose << std::endl;
    std::cout << "Goal:" << std::endl;
    std::cout << goalPose << std::endl;
    std::cout << "L1: " << configRes[0][2] << " L2: " << configRes[1][2] << " L3: " << configRes[2][2] << std::endl;

    auto start = std::chrono::steady_clock::now();
    robot.updateDiscreteFrames();
    auto end = std::chrono::steady_clock::now();
    std::cout << "Discrete frames time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " µs" << std::endl;


    }

    return 0;
}