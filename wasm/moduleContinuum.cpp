#include <iostream>
#include <chrono>
#include <unistd.h>


// Uncomment for EMSCRIPTEN ********
#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "VariableCurvatureRobot.h"
#include "Continuum/Continuum.h"
#include "Continuum/ContinuumUtils.h"

//#include "Trajectory/Trajectory.h"
#include "../SpiralData.h"

struct VelData{
    double pos;
    double minError;
    double vel;
    double lastTime;
    VelData(double pos, double minError, double vel, double lastTime) : pos(pos), minError(minError), vel(vel), lastTime(lastTime){}

    double updatePos(double goal, double time){
        double error = goal - pos;
        if(abs(error) > minError){
            double deltaTime = time - lastTime;
            if(( goal - pos ) > 0.0) pos += vel * deltaTime;
            else pos -= vel * deltaTime;
        }
        lastTime = time;
        return pos;
    }
};


//Trajectory traj;
VariableCurvatureRobot robot(3);

Eigen::Matrix4d target_transform = Eigen::Matrix4d::Identity();

auto startTimestamp = std::chrono::steady_clock::now();

int loops_done = 0;
int ptsIter = 0;

std::vector<double> goal;

//std::vector<double> globalPos = { -0.2654, 4.993, 15.6814 };
//std::vector<double> globalPos = { 15.6814, 4.993, -0.2654 };
std::vector<double> globalPos = { 7.5, 0.0, 0.5};
Eigen::Vector3d lookTarget = {15.0, 0.0, 0.0};
Eigen::Vector3d lookTargetNow = lookTarget;

VelData xSmoothen(globalPos[0], 0.05, 2.0, 0.0);
VelData ySmoothen(globalPos[1], 0.05, 2.0, 0.0);
VelData zSmoothen(globalPos[2], 0.05, 2.0, 0.0);
VelData targetSmoothen(lookTarget[0], 0.05, 2.0, 0.0);


void setXYZ(double x, double y, double z){
    globalPos[0] = x;
    globalPos[1] = y;
    globalPos[2] = z;
}



void setTargetX(double x){
    lookTarget[0] = x;
}

double getTargetX(){
    return lookTargetNow[0];
}


void animationStep(){
//    auto nowTimestamp = std::chrono::steady_clock::now();
//    double t = double(std::chrono::duration_cast<std::chrono::microseconds>(nowTimestamp - startTimestamp).count()) / 1000000.0;
//    while(t-8.0 > ((loops_done + 1) * traj.getTotalTime())){
//        ++loops_done;
//        traj.currentSegmentNumber = 0;
//    }
//    auto timeTransl = t - ( loops_done * traj.getTotalTime() );
//    timeTransl -= 8.0;
//    if(timeTransl < 0.0) timeTransl = 0.0;

//    auto [x0, y0, z0] = pathpoints[ptsIter];
//    auto [x0, y0, z0] = globalPos;
    Eigen::Matrix4d effectorTargetT = target_transform;

//    Eigen::Matrix4d effectorTargetT = Eigen::Matrix4d::Identity();

//    effectorTargetT = robot.effectorT;

    effectorTargetT << 0.0, 0.0, 1.0, 0.0,
                       1.0,  0.0, 0.0, 0.0,
                       0.0,  1.0, 0.0, 0.0,
                       0.0,  0.0, 0.0, 1.0;


//    effectorTargetT = effectorTargetT * ContinuumUtils::rotY(-M_PI * 1.0 / 3.0);

//    effectorTargetT(0,3) = x0;
//    effectorTargetT(1,3) = y0;
//    effectorTargetT(2,3) = z0;

//    globalPos[0] += 0.005;

    auto nowTimestamp = std::chrono::steady_clock::now();
    double t = double(std::chrono::duration_cast<std::chrono::microseconds>(nowTimestamp - startTimestamp).count()) / 1000000.0;

    effectorTargetT(0,3) = xSmoothen.updatePos(globalPos[0], t);
    effectorTargetT(1,3) = ySmoothen.updatePos(globalPos[1], t);
    effectorTargetT(2,3) = zSmoothen.updatePos(globalPos[2], t);

//    effectorTargetT(0,3) = globalPos[0];
//    effectorTargetT(1,3) = globalPos[1];
//    effectorTargetT(2,3) = globalPos[2];

    Eigen::Matrix4d effectorLaserTranslate = Eigen::Matrix4d::Identity();
    effectorLaserTranslate(1,3) = -0.5;
    effectorTargetT = effectorTargetT * effectorLaserTranslate;

    Eigen::Vector3d globalYAxis;
    globalYAxis << 0.0, 1.0, 0.0;

    lookTargetNow[0] = targetSmoothen.updatePos(lookTarget[0],t);

    Eigen::Vector3d newZAxis;
    newZAxis = (lookTargetNow - effectorTargetT.block(0,3,3,1));
    newZAxis /= newZAxis.norm();

    Eigen::Vector3d newYAxis;
    newYAxis = newZAxis.cross(globalYAxis);
    newYAxis /= newYAxis.norm();

    Eigen::Vector3d newXAxis;
    newXAxis = newYAxis.cross(newZAxis);
    newXAxis /= newXAxis.norm();

    effectorTargetT.block(0,0,3,1) = newXAxis;
    effectorTargetT.block(0,1,3,1) = newYAxis;
    effectorTargetT.block(0,2,3,1) = newZAxis;

    effectorTargetT = effectorTargetT * effectorLaserTranslate.inverse();

    // IK Solution
     auto res = robot.ikWorld(effectorTargetT);
//     Eigen::Matrix4d resT = robot.fk(res);


//    std::cout << "Diff Pose - Goal:" << std::endl;
//    std::cout << (resT.block(0,3,3,1) - effectorTargetT.block(0,3,3,1)).norm() << std::endl;

    robot.updateDiscreteFrames();

//    if(ptsIter < 350) ++ptsIter;
}

std::vector<double> getTubeSegmentTransform(int tubeID, int segmentID){
   return robot.getTubeSegmentTransform(tubeID, segmentID);
}

std::vector<double> getRobotBaseTransform(){
    return robot.getVectorFromMatrix(robot.robotBase);
}

std::vector<double> getRobotEffectorTransform(){
    return robot.getVectorFromMatrix(robot.getEffectorWorld());
}

std::vector<double> getLaserTransform(){
    Eigen::Matrix4d laserTransform = Eigen::Matrix4d::Identity();
//    laserTransform(1,3) = -0.530;
    laserTransform(1,3) = -0.500;
    Eigen::Matrix4d res = robot.getEffectorWorld()*laserTransform;
    return robot.getVectorFromMatrix(res);
}




int _main() {
//int main() {
//    std::vector<std::array<double,3>> config = {
//            { 0.552649229, -83.24960826, 1.237827 },
//            { 0.5579576148, 85.24228818, 1.308040 },
//            { 0.102175075, -1.895120665, 2.699483 }
//    };

//    target_transform.block(0,0,3,3) = robot.robotBase.block(0,0,3,3);

//    std::vector<std::array<double,3>> config = {
//        { -0.164145, -96.0445, 5.88967 },
//        { -0.188756, 97.121, 5.49263 },
//        { 0.0389489, -1.19957, 6.06818 }
//    };

//    lookTarget << 15.0, 0.0, 0.0;



//    std::vector<std::array<double,3>> config = {
//            { -0.164145, -96.0445, 2.88967 },
//            { -0.188756, 97.121, 2.49263 },
//            { 0.0389489, -1.19957, 2.06818 }
//    };

//    std::vector<std::array<double,3>> config = {
//            { 0.001, 0.0, 2.5 },
//            { 0.001, 0.0, 2.0 },
//            { 0.001, 0.0, 3.0 }
//    };

    std::vector<std::array<double,3>> config = {
            { 0.001, 0.0, 2.5 },
            { 0.001, 0.0, 2.5 },
            { 0.001, 0.0, 2.5 }
    };

    robot.fk(config);
//    animationStep();

//    target_transform = target_transform * ContinuumUtils::rotZ(M_PI_2);
//    robot.ik(target_transform);
//    target_transform = target_transform * ContinuumUtils::rotZ(M_PI_2);
//    robot.ik(target_transform);

    robot.updateDiscreteFrames();
    std::cout << "End effector at input configuration:" <<std::endl;
    std::cout << robot.getEffectorWorld() << std::endl;

//0.352674  0.931883 0.0849416  0.258927
//9 -0.905976  0.362758 -0.218205 -0.665152
//9 -0.234155         0  0.972199   6.01184

//    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//    Eigen::MatrixXd pts = VariableCurvatureUtils::spiralCompute();
//    int numPts = pts.size() / 4;
//    Eigen::MatrixXd ptsTransf = T * pts;
//    Eigen::MatrixXd ptsReady(3, numPts);
//    ptsReady = ptsTransf.block(0,0,3, numPts);
//    double vel = 0.010;
//    traj.computeMultiSegment(ptsReady, vel);

//    animationStep();

//    auto m = getTubeSegmentTransform(0, 12);
//    std::cout << std::endl;
//    for(auto el : m) std::cout << el << " ";
    return 0;
}

// Uncomment for EMSCRIPTEN ********
EMSCRIPTEN_BINDINGS(module)
{
   emscripten::function("animationStep", &animationStep);
   emscripten::function("getTubeSegmentTransform", &getTubeSegmentTransform);
   emscripten::function("getRobotBaseTransform", &getRobotBaseTransform);
   emscripten::function("getRobotEffectorTransform", &getRobotEffectorTransform);
   emscripten::function("getLaserTransform", &getLaserTransform);
   emscripten::function("setXYZ", &setXYZ);
   emscripten::function("setTargetX", &setTargetX);
   emscripten::function("getTargetX", &getTargetX);
//   emscripten::function("setXYZ", &setXYZ);
   emscripten::function("main", &_main);
   emscripten::register_vector<double>("vector<double>");
}


//std::map<std::string,std::vector<double>> fnGetMovingObjects(){
    //return robot.map;
//}

//std::vector<MyObject> fnGetMovingObjects(){
    //return robot.objectVec;
//}


//// 600 objects

//map 16 strings and 16 x 16 double


