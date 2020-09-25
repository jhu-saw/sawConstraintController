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

#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

simpleTeleop::simpleTeleop(const std::string & componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

void simpleTeleop::Init() {
    SetupRobot();
    SetupVF();

    StateTable.AddData(mJacobian,"Jacobian");
    StateTable.AddData(mMeasuredCartesianPosition,"MeasuredCartesianPosition");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("ProvidesSimpleRobot");
    if (interfaceProvided){
        interfaceProvided->AddCommandWrite(&simpleTeleop::ServoCartesianPosition, this, "ServoCartesianPosition");
        interfaceProvided->AddCommandReadState(StateTable, mMeasuredCartesianPosition, "GetMeasuredCartesianPosition");
    }

    std::cout << "Robot running"<< std::endl;
}

void simpleTeleop::SetupRobot() {
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
    ForwardKinematics(mJointPosition);
    mMeasuredCartesianPosition.SetReferenceFrame("map");
    mMeasuredCartesianPosition.Valid() = true;
}

void simpleTeleop::SetupVF() {
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
    mJointLimitsConstraint.LowerLimits.Assign(-150.0, -150.0, -150.0, -2.0, -2.0, -2.0).Multiply(cmn_mm);
    mJointLimitsConstraint.UpperLimits.SetSize(mNumJoints);
    mJointLimitsConstraint.UpperLimits.Assign(150.0, 150.0, 150.0, 2.0, 2.0, 2.0).Multiply(cmn_mm);
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

   // mesh constraint
   mMeshFile = msh3Mesh(true); // convert mesh unit from mm to m
   cmnPath path;
   path.AddRelativeToCisstShare("/models/meshes");
   std::string meshFile = "Cube.STL";
   std::string fullPath = path.Find(meshFile);
   if (mMeshFile.LoadMeshFromSTLFile(fullPath)==-1){
       CMN_LOG_CLASS_RUN_ERROR << "Cannot load STL file" << std::endl;
       cmnThrow("Cannot load STL file");
   }
   else{
       mMesh.Name = "Mesh";
       mMesh.NumTrianglesInNode = 5;
       mMesh.DiagonalDistanceOfNode = 0.005; // divide a node whenver distance has reached
       mMesh.BoundingDistance = 0.005; // bounding distance for intersection detection
       mMesh.NumJoints = mNumJoints;
       mMesh.KinNames.clear(); // sanity
       // use the names defined above to relate kinematics data
       mMesh.KinNames.push_back("MeasuredKinematics");

       if (!mController->SetVFData(mMesh))
       {
           mController->VFMap.insert(std::pair<std::string, mtsVFMesh*>(mMesh.Name, new mtsVFMesh(mMesh.Name, &mMesh, mMeshFile)));
       }
   }
}

void simpleTeleop::ForwardKinematics(vctDoubleVec& jointPosition) {
    mMeasuredCartesianPosition.Position().Translation() = jointPosition.Ref(3,0);
}

void simpleTeleop::Run() {
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // acquire sensor/kinematics information
    // activate/deactivate constraints (or select desired behaviours) if needed

    // solve for next movement
    vctDoubleVec dq;
    nmrConstraintOptimizer::STATUS optimizerStatus =Solve(dq);

    if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
        mJointPosition += dq.Ref(6,0);
        // move
        ForwardKinematics(mJointPosition);
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

void simpleTeleop::UpdateOptimizerKinematics() {
    // update cartesian position and jacobian
    mMeasuredKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredKinematics.JointState->SetPosition(mJointPosition);

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

nmrConstraintOptimizer::STATUS simpleTeleop::Solve(vctDoubleVec &dq) {
    // update optimizer kinematics
    UpdateOptimizerKinematics();

    // update sensor data if needed
    // update vf data if needed

    // update optimizer
    mController->UpdateOptimizer(StateTable.GetAveragePeriod());

    // solve
    return mController->Solve(dq);
}

void simpleTeleop::ServoCartesianPosition(const vctFrm4x4 & newGoal) {
    mGoalKinematics.Frame.FromNormalized(newGoal);
}
