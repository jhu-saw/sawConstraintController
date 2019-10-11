//
// Created by max on 2019-10-11.
//

#include <cstdio>
#include "simpleRobot.h"

simpleRobot::simpleRobot(const std::string & componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    init();
}

void simpleRobot::init() {
    mNumDof = 6;
    mNumJoints = 6;
    mJacobian = vctDoubleMat(mNumDof, mNumJoints);
    mJacobian.SetAll(0.0);

    mCartesianPosition.Identity();

    StateTable.AddData(mJacobian,"Jacobian");
    StateTable.AddData(mCartesianPosition,"CartesianPosition");
}

void simpleRobot::Run() {
    // process the events received
    ProcessQueuedEvents();

    mCartesianPosition.at(0,3) += 0.01;
    std::cout << mCartesianPosition.ToString() << std::endl;
}