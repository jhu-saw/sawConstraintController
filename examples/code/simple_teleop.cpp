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
#include "simple_teleop.h"

#include <cisstCommon/cmnConstants.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

simpleTeleop::simpleTeleop(const std::string & componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    init();
}

void simpleTeleop::init() {
    setupRobot();
    setupVF();

    StateTable.AddData(mJacobian,"Jacobian");
    StateTable.AddData(mMeasuredCartesianPosition,"MeasuredCartesianPosition");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("ProvidesSimpleRobot");
    if (interfaceProvided){
        interfaceProvided->AddCommandWrite(&simpleTeleop::servoCartesianPosition, this, "ServoCartesianPosition");
        interfaceProvided->AddCommandWrite(&simpleTeleop::transformationCallback, this, "TransformationCallback");
        interfaceProvided->AddCommandReadState(StateTable, mMeasuredCartesianPosition, "GetMeasuredCartesianPosition");
    }
}

void simpleTeleop::setupRobot() {
    mNumDof = 6;
    mNumJoints = 6;

    // joint values
    mJointPosition.SetSize(mNumJoints);
    mJointPosition.SetAll(0.0);
    mJointPosition.Ref(3,0).Assign(-30.0,-30.0,-30.0).Multiply(cmn_mm);

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

void simpleTeleop::setupVF() {
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
    mController->AddVFFollow(mTeleopObjective);

    // joint limit constraint
    mJointLimitsConstraint.Name = "Joint Limit";
    mJointLimitsConstraint.IneqConstraintRows = 2 * mNumJoints;
    mJointLimitsConstraint.LowerLimits.SetSize(mNumJoints);
    mJointLimitsConstraint.LowerLimits.Assign(-150.0, -150.0, -150.0, -2.0, -2.0, -2.0).Multiply(1E-3);
    mJointLimitsConstraint.UpperLimits.SetSize(mNumJoints);
    mJointLimitsConstraint.UpperLimits.Assign(150.0, 150.0, 150.0, 2.0, 2.0, 2.0).Multiply(1E-3);
    mJointLimitsConstraint.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mJointLimitsConstraint.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFLimitsConstraint.cpp
    mController->AddVFLimits(mJointLimitsConstraint);

   // plane constraint
   mPlaneConstraint.Name = "PlaneConstraint";
   mPlaneConstraint.IneqConstraintRows = 1;
   mPlaneConstraint.Normal.Assign(1.0,0.0,1.0).NormalizedSelf();
   mPlaneConstraint.PointOnPlane.Assign(0.0, 0.0, -5.0);
   mPlaneConstraint.NumJoints = mNumJoints;
   // use the names defined above to relate kinematics data
   mPlaneConstraint.KinNames.push_back("MeasuredKinematics"); // need measured kinematics according to mtsVFPlane.cpp
   mController->AddVFPlane(mPlaneConstraint);

   // cylindrical constraint
   vct3 origin(-55,-30,0), left(15,-27,3),right(-15,-27,3),end(0,27,3);
   mNerveLeft.Name = "Nerve Left";
   mNerveLeft.IneqConstraintRows = 1;
   mNerveLeft.Axis.Assign(left-end);
   mNerveLeft.Point.Assign(left-origin);
   mNerveLeft.Radius = 5.0;
   mNerveLeft.NumJoints = mNumJoints;
   mNerveLeft.KinNames.clear(); // sanity
   // use the names defined above to relate kinematics data
   mNerveLeft.KinNames.push_back("MeasuredKinematics");
   mController->AddVFCylinder(mNerveLeft);

   mNerveRight.Name = "Nerve Right";
   mNerveRight.IneqConstraintRows = 1;
   mNerveRight.Axis.Assign(right-end);
   mNerveRight.Point.Assign(right-origin);
   mNerveRight.Radius = 5.0;
   mNerveRight.NumJoints = mNumJoints;
   mNerveRight.KinNames.clear(); // sanity
   // use the names defined above to relate kinematics data
   mNerveRight.KinNames.push_back("MeasuredKinematics");
   mController->AddVFCylinder(mNerveRight);
}

void simpleTeleop::forwardKinematics(vctDoubleVec& jointPosition) {
    // TODO: how to access part of the vector?
    mMeasuredCartesianPosition.Position().Translation() = jointPosition.Ref(3,0);
    // TODO: update rotation
}

void simpleTeleop::Run() {
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // acquire sensor/kinematics information
    // activate/deactivate constraints (or select desired behaviours) if needed

    // solve for next movement
    vctDoubleVec dq;
    nmrConstraintOptimizer::STATUS optimizerStatus =solve(dq);

    if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
        mJointPosition += dq.Ref(6,0);
        std::cout << 1.0/StateTable.GetAveragePeriod() << std::endl;
        // move
        forwardKinematics(mJointPosition);
    }
    else{
        std::cout << "No solution found" << std::endl;
        std::cout << optimizerStatus << std::endl;

        if (optimizerStatus==nmrConstraintOptimizer::STATUS::NMR_INEQ_CONTRADICTION){
            std::cout << "inqeuality rows " << mController->Optimizer.GetIneqConstraintMatrix().rows() << "cols "<<mController->Optimizer.GetIneqConstraintMatrix().cols()<< std::endl;
            std::cout << "inqeuality vector " << mController->Optimizer.GetIneqConstraintVector().size() << std::endl;
        }

    }
}

void simpleTeleop::updateOptimizerKinematics() {
    // update cartesian position and jacobian
    mMeasuredKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredKinematics.JointState->SetPosition(mJointPosition);

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

nmrConstraintOptimizer::STATUS simpleTeleop::solve(vctDoubleVec &dq) {
    // update optimizer kinematics
    updateOptimizerKinematics();

    // update sensor data if needed
    // update vf data if needed

    // update optimizer
    mController->UpdateOptimizer(StateTable.GetAveragePeriod());

    // solve
    return mController->Solve(dq);
}

void simpleTeleop::servoCartesianPosition(const vctFrm4x4 & newGoal) {
    mGoalKinematics.Frame.FromNormalized(newGoal);
}

void simpleTeleop::transformationCallback(const vctFrm4x4 &transformation)
{
    std::cout << "-----------------------\n";
    std::cout << "Transform received!!!!! " << std::endl;
    std::cout << "Jacobian: \n" << mJacobian << std::endl;
    std::cout << "Transformed Jacobian: \n" << vctMat(transformation.Rotation())*mJacobian.Ref(3,3,0,0) << std::endl;
    std::cout << "Tip: \n" << mMeasuredCartesianPosition.Position() << std::endl;
    std::cout << "Transformed Tip: \n" << vctFrm3(transformation.ApplyTo(vctFrm4x4(mMeasuredCartesianPosition.Position()))) << std::endl;
    std::cout << "-----------------------\n";
}
