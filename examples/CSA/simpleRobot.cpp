/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zhaoshuo Li
  Created on: 2019-10-21

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cstdio>
#include "simpleRobot.h"

#include <cisstCommon/cmnConstants.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

simpleRobot::simpleRobot(const std::string & componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    init();
}

void simpleRobot::init() {
    setupRobot();
    setupVF();

    StateTable.AddData(mJacobian,"Jacobian");
    StateTable.AddData(mMeasuredCartesianPosition,"MeasuredCartesianPosition");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("ProvidesSimpleRobot");
    if (interfaceProvided){
        interfaceProvided->AddCommandWrite(&simpleRobot::servoCartesianPosition, this, "ServoCartesianPosition");
        interfaceProvided->AddCommandReadState(StateTable, mMeasuredCartesianPosition, "GetMeasuredCartesianPosition");
    }
}

void simpleRobot::setupRobot() {
    mNumDof = 6;
    mNumJoints = 6;

    // joint values
    mJointPosition.SetSize(mNumJoints);
    mJointPosition.SetAll(0.0);

    // fixed jacobian
    mJacobian.SetSize(mNumDof,mNumJoints);
    mJacobian.SetAll(0.0);
    for (int i = 0; i < mNumJoints; i ++){ // one to one mapping of joint to cartesian position
        mJacobian[i][i] = 1.0;
    }

    // initialize cartesian
    forwardKinematics(mJointPosition);
    mMeasuredCartesianPosition.SetReferenceFrame("map");
    mMeasuredCartesianPosition.Valid() = true;
}

void simpleRobot::setupVF() {
    // initialize controller
    mController = new mtsVFController(mNumJoints, mtsVFBase::JPOS);

    // robot related data
    mMeasuredKinematics.Name="MeasuredKinematics";
    mMeasuredKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());
    mMeasuredKinematics.Jacobian.SetSize(mNumDof, mNumJoints);
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredJoint.SetSize(mNumJoints);
    mMeasuredJoint.SetPosition(mJointPosition);
    mMeasuredKinematics.JointState = &mMeasuredJoint;

    mGoalKinematics.Name="GoalKinematics";
    mGoalKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());

    // objective
    mTeleopObjective.Name = "Teleop";
    mTeleopObjective.ObjectiveRows = mNumJoints;
    mTeleopObjective.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mTeleopObjective.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFFollow.cpp
    mTeleopObjective.KinNames.push_back("GoalKinematics"); // goal kinematics needs to be second

    // joint limit constraint
    mJointLimitsConstraint.Name = "Joint Limit";
    mJointLimitsConstraint.LowerLimits.SetSize(mNumJoints);
    mJointLimitsConstraint.LowerLimits.Assign(-10.0, -10.0, -10.0, -2.0, -2.0, -2.0);
    mJointLimitsConstraint.UpperLimits.SetSize(mNumJoints);
    mJointLimitsConstraint.UpperLimits.Assign(10.0, 10.0, 10.0, 2.0, 2.0, 2.0);
    mJointLimitsConstraint.IneqConstraintRows = 2 * mNumJoints;
    mJointLimitsConstraint.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mJointLimitsConstraint.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFLimitsConstraint.cpp

    // plane constraint
    mPlaneConstraint.Name = "PlaneConstraint";
    mPlaneConstraint.IneqConstraintRows = 1;
    mPlaneConstraint.Normal.Assign(0.0,0.0,1.0);
    mPlaneConstraint.PointOnPlane.Assign(0.0, 0.0, -5.0);
    // use the names defined above to relate kinematics data
    mPlaneConstraint.KinNames.push_back("MeasuredKinematics"); // need measured kinematics according to mtsVFPlane.cpp
    // slack
    mPlaneConstraint.NumSlacks = 1;
    mPlaneConstraint.SlackCosts = 1.0;
    mPlaneConstraint.SlackLimits.SetSize(1);
    mPlaneConstraint.SlackLimits.Assign(vctDouble1(1.0));

    // add objective and constraint to optimizer
    // first, we check if we can set the data. If not, we insert it.
    if (!mController->SetVFData(mTeleopObjective, typeid(mtsVFFollow)))
    {
        // Adds a new virtual fixture to the active vector
        mController->VFMap.insert(std::pair<std::string,mtsVFFollow *>(mTeleopObjective.Name,new mtsVFFollow(mTeleopObjective.Name,new mtsVFDataBase(mTeleopObjective))));
        // Increment users of each kinematics and sensor object found
        mController->IncrementUsers(mTeleopObjective.KinNames, mTeleopObjective.SensorNames);
    }
    if (!mController->SetVFData(mJointLimitsConstraint, typeid(mtsVFLimitsConstraint)))
    {
        // Adds a new virtual fixture to the active vector
        mController->VFMap.insert(std::pair<std::string,mtsVFLimitsConstraint *>(mJointLimitsConstraint.Name,new mtsVFLimitsConstraint(mJointLimitsConstraint.Name,new mtsVFDataJointLimits(mJointLimitsConstraint))));
        // Increment users of each kinematics and sensor object found
        mController->IncrementUsers(mJointLimitsConstraint.KinNames, mJointLimitsConstraint.SensorNames);
    }
    if (!mController->SetVFData(mPlaneConstraint, typeid(mtsVFPlane)))
    {
        mController->VFMap.insert(std::pair<std::string, mtsVFPlane*>(mPlaneConstraint.Name, new mtsVFPlane(mPlaneConstraint.Name, new mtsVFDataPlane(mPlaneConstraint))));
        mController->IncrementUsers(mPlaneConstraint.KinNames, mPlaneConstraint.SensorNames);
    }

}

void simpleRobot::forwardKinematics(vctDoubleVec& jointPosition) {
    // TODO: how to access part of the vector?
    mMeasuredCartesianPosition.Position().Translation() = jointPosition.Ref(3,0);
    // TODO: update rotation
}

void simpleRobot::Run() {
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // activate/deactivate constraints (or select desired behaviours) if needed

    // solve for next movement
    vctDoubleVec dq;
    nmrConstraintOptimizer::STATUS optimizerStatus =solve(dq);

    if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
        mJointPosition += dq.Ref(6,0);
        // move
        forwardKinematics(mJointPosition);
    }
    else{
        std::cout << "No solution found" << std::endl;
        std::cout << optimizerStatus << std::endl;

    }
}

void simpleRobot::updateOptimizerKinematics() {
    // update cartesian position and jacobian
    mMeasuredKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredKinematics.JointState->SetPosition(mJointPosition);

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

nmrConstraintOptimizer::STATUS simpleRobot::solve(vctDoubleVec &dq) {
    // update optimizer kinematics
    updateOptimizerKinematics();

    // update sensor data if needed
    // update vf data if needed

    // update optimizer
    mController->UpdateOptimizer(StateTable.GetAveragePeriod());

    // solve
    return mController->Solve(dq);
}

void simpleRobot::servoCartesianPosition(const vctFrm4x4 & newGoal) {
    mGoalKinematics.Frame.FromNormalized(newGoal);
}
