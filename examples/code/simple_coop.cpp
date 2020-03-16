/*
  Author(s):  Max Zhaoshuo Li
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
#include "simple_coop.h"

#include <cisstCommon/cmnConstants.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

simpleCoop::simpleCoop(const std::string & componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

void simpleCoop::Init() {
    SetupRobot();
    SetupVFBehaviour();

    StateTable.AddData(mJacobian,"Jacobian");
    StateTable.AddData(mMeasuredCartesianPosition,"MeasuredCartesianPosition");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("ProvidesSimpleRobot");
    if (interfaceProvided){
        interfaceProvided->AddCommandWrite(&simpleCoop::ServoCartesianForce, this, "ServoCartesianForce");
        interfaceProvided->AddCommandReadState(StateTable, mMeasuredCartesianPosition, "GetMeasuredCartesianPosition");
    }
    std::cout << "Robot running"<< std::endl;
}

void simpleCoop::SetupRobot() {
    mNumOutput = 6;
    mNumJoints = 6;
    mNumFTDoF = 6;

    // joint values
    mJointPosition.SetSize(mNumJoints);
    mJointPosition.SetAll(0.0);

    // fixed jacobian
    mJacobian.SetSize(mNumOutput,mNumJoints);
    mJacobian.SetAll(0.0);
    for (int i = 0; i < mNumJoints; i ++){ // one to one mapping of joint to cartesian position
        mJacobian[i][i] = 1.0;
    }

    // initialize cartesian
    ForwardKinematics(mJointPosition);
    mMeasuredCartesianPosition.SetReferenceFrame("map");
    mMeasuredCartesianPosition.Valid() = true;
}

void simpleCoop::SetupVFBehaviour() {
    // initialize controller
    mController = new mtsVFController(mNumJoints, mtsVFBase::JVEL);

    // robot related data
    mMeasuredKinematics.Name="MeasuredKinematics";
    mMeasuredKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());
    mMeasuredKinematics.Jacobian.SetSize(mNumOutput, mNumJoints);
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredJoint.SetSize(mNumJoints);
    mMeasuredJoint.SetPosition(mJointPosition);
    mMeasuredKinematics.JointState = &mMeasuredJoint;

    mGoalForceValues.Name="GoalForce";
    mGoalForceValues.Values.SetSize(mNumFTDoF);
    mGoalForceValues.Values.SetAll(0.0);

    // objective
    mCoopObjective.Name = "Coop";
    mCoopObjective.ObjectiveRows = mNumOutput; // num of output dimensions
    mCoopObjective.Gain.SetSize(mNumOutput);
    mCoopObjective.Gain.SetAll(0.0);

    mCoopObjective.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mCoopObjective.KinNames.push_back("MeasuredKinematics");
    mCoopObjective.SensorNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mCoopObjective.SensorNames.push_back("GoalForce");
    mController->AddVFSensorCompliance(mCoopObjective);

    // plane constraint
    mPlaneConstraint.Name = "PlaneConstraint";
    mPlaneConstraint.IneqConstraintRows = 1;
    mPlaneConstraint.Normal.Assign(0.0,0.0,1.0);
    mPlaneConstraint.PointOnPlane.Assign(0.0, 0.0, -2.0);
    mPlaneConstraint.NumJoints = mNumJoints;
    // use the names defined above to relate kinematics data
    mPlaneConstraint.KinNames.push_back("MeasuredKinematics"); // need measured kinematics according to mtsVFPlane.cpp
    // slack
    mPlaneConstraint.NumSlacks = 1;
    mPlaneConstraint.SlackCosts.SetSize(1);
    mPlaneConstraint.SlackCosts.Assign(vctDouble1(1.0));
    mPlaneConstraint.SlackLimits.SetSize(1);
    mPlaneConstraint.SlackLimits.Assign(vctDouble1(1.0));
    mController->AddVFPlane(mPlaneConstraint);

    // joint limit constraint
    mJointLimitsConstraint.Name = "Joint Limit";
    mJointLimitsConstraint.LowerLimits.SetSize(mNumJoints);
    mJointLimitsConstraint.LowerLimits.Assign(-0.1, -0.1, -0.1, -0.01, -0.01, -0.01);
    mJointLimitsConstraint.UpperLimits.SetSize(mNumJoints);
    mJointLimitsConstraint.UpperLimits.Assign(0.1, 0.1, 0.1, 0.01, 0.01, 0.01);
    mJointLimitsConstraint.IneqConstraintRows = 2 * mNumJoints;
    mJointLimitsConstraint.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mJointLimitsConstraint.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFLimitsConstraint.cpp
    mController->AddVFLimits(mJointLimitsConstraint);
}

void simpleCoop::ForwardKinematics(vctDoubleVec& jointPosition) {
    mMeasuredCartesianPosition.Position().Translation() = jointPosition.Ref(3,0);
    // TODO: update rotation
}

void simpleCoop::Run() {
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // activate/deactivate constraints (or select desired behaviours) if needed

    // solve for next movement
    vctDoubleVec dq;
    nmrConstraintOptimizer::STATUS optimizerStatus = RunBehaviour(dq);

    if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
        mJointPosition += dq.Ref(mNumJoints,0).Multiply(StateTable.GetAveragePeriod()); // convert v to q
        // move
        ForwardKinematics(mJointPosition);

        // reset
        mGoalForceValues.Values.SetAll(0.0);
    }
    else{
        CMN_LOG_CLASS_RUN_ERROR << "No solution found" << std::endl;
        CMN_LOG_CLASS_RUN_ERROR << optimizerStatus << std::endl;

        if (optimizerStatus==nmrConstraintOptimizer::STATUS::NMR_INEQ_CONTRADICTION){
            CMN_LOG_CLASS_RUN_ERROR << "inqeuality rows " << mController->Optimizer.GetIneqConstraintMatrix().rows() << "cols "<<mController->Optimizer.GetIneqConstraintMatrix().cols()<< std::endl;
            CMN_LOG_CLASS_RUN_ERROR << "inqeuality vector " << mController->Optimizer.GetIneqConstraintVector().size() << std::endl;
        }
    }
}

void simpleCoop::UpdateOptimizerKinematics() {
    // update cartesian position and jacobian
    mMeasuredKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredKinematics.JointState->SetPosition(mJointPosition);

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
}

void simpleCoop::UpdateOptimizerSensor()
{
    // update controller stored sensor
    mController->SetSensor(mGoalForceValues);
}

nmrConstraintOptimizer::STATUS simpleCoop::RunBehaviour(vctDoubleVec &dq) {
    // update optimizer kinematics
    UpdateOptimizerKinematics();
    // update sensor data
    UpdateOptimizerSensor();

    // update vf data if needed

    // update optimizer
    mController->UpdateOptimizer(StateTable.GetAveragePeriod());

    // solve
    return mController->Solve(dq);
}

void simpleCoop::ServoCartesianForce(const mtsDoubleVec & newGoal) {
    mGoalForceValues.Values.Assign(newGoal);
    // change gain
    mCoopObjective.Gain.SetAll(1.0);
}
