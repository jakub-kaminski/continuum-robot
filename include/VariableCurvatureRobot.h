#pragma once

#include <map>
//#include "Eigen/Dense"
//#include "Eigen/Core"

#include "Continuum/Continuum.h"
#include "Continuum/ContinuumUtils.h"

class VariableCurvatureRobot : public ContinuumRobotInterface {
public:
    VariableCurvatureRobot(int numTubes) : ContinuumRobotInterface(numTubes) {}
};