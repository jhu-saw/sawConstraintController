//
// Created by max on 2019-10-11.
//
#include <cstdio>
#include "simpleRobot.h"

#include <cisstCommon/cmnConstants.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

// required to implement the class services, see cisstCommon
CMN_IMPLEMENT_SERVICES_DERIVED(simpleRobot, mtsTaskPeriodic);

simpleRobot::simpleRobot(const std::string & componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    init();
}

void simpleRobot::init() {
    setupRobot();
    setupVF();

    StateTable.AddData(mJacobian,"Jacobian");
    StateTable.AddData(mCartesianPosition,"CartesianPosition");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("ProvidesSimpleRobot");
    if (interfaceProvided){
        interfaceProvided->AddCommandWrite(&simpleRobot::servoCartesianRelative, this, "ServoCartesianRelative");
    }
}

void simpleRobot::setupRobot() {
    mNumDof = 6;
    mNumJoints = 6;

    // joint values
    mJointPosition.SetSize(mNumJoints);
    mJointPosition.SetAll(1.0);

    // fixed jacobian
    mJacobian.SetSize(mNumDof,mNumJoints);
    mJacobian.SetAll(0.0);
    for (int i = 0; i < mNumJoints; i ++){ // one to one mapping of joint to cartesian position
        mJacobian[i][i] = 1.0;
    }

    // initialize cartesian
    mCartesianPosition.Identity();
}

void simpleRobot::setupVF() {
    // initialize controller
    mController = new mtsVFController(mNumJoints, mtsVFBase::JPOS);

    // robot related data
    mMeasuredKinematics.Name="MeasuredKinematics";
    mMeasuredKinematics.Frame.FromNormalized(mCartesianPosition);
    mMeasuredKinematics.Jacobian.SetSize(mNumDof, mNumJoints);
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredJoint.SetSize(mNumJoints);
    mMeasuredJoint.SetPosition(mJointPosition);
    mMeasuredKinematics.JointState = &mMeasuredJoint;

    mGoalKinematics.Name="GoalKinematics";
    mGoalKinematics.Frame.FromNormalized(mCartesianPosition);

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
    mCartesianPosition.Translation() = jointPosition.XYZ();
    // TODO: update rotation
}

void simpleRobot::Run() {
    // process the events received
    ProcessQueuedEvents();

    // solve for next movement
    vctDoubleVec dq;
    nmrConstraintOptimizer::STATUS optimizerStatus =solve(dq);

    if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
        std::cout << "solved successfully \n";
        mJointPosition += dq;
        // move
        forwardKinematics(mJointPosition);
    }
    else{
        std::cout << "No solution found" << std::endl;
        std::cout << optimizerStatus << std::endl;

    }
    std::cout << "Measured CP \n" << mCartesianPosition << std::endl;
}

void simpleRobot::updateKinematics() {
    // update cartesian position and jacobian
    mMeasuredKinematics.Frame.FromNormalized(mCartesianPosition);
    mMeasuredKinematics.Jacobian.Assign(mJacobian);

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

nmrConstraintOptimizer::STATUS simpleRobot::solve(vctDoubleVec &dq) {
    std::cout << "update kinematics \n";
    // update kinematics
    updateKinematics();

    // update optimizer
    std::cout << "update optimizer \n";
    mController->UpdateOptimizer(StateTable.GetAveragePeriod());

    // update vf data if needed

    // solve
    return mController->Solve(dq);
}

void simpleRobot::servoCartesianRelative(const mtsFrm4x4 & newGoal) {
    mGoalKinematics.Frame.FromNormalized(newGoal);
}