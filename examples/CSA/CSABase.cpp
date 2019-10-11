//
// Created by max on 2019-10-10.
//

#include "CSABase.h"

CSABase::CSABase(const int &_numDoF, const int &_numJoints, mtsComponent *_component):
    mNumDoF(_numDoF),
    mNumJoints(_numJoints),
    mRobot(_component)
{
    init();
}

void CSABase::init() {
    // initialize other parameters
    initParameters();
    // call robot setup interfaces
    setupRobotInterface();
    // call vf setup interfaces
    setupVFInterface();
    // initialize controller
    initVFController(mtsVFBase::JPOS);
}

void CSABase::initParameters() {
    // initialize necessary parameters here
    initKinematicName();
    initSensorName();
}

void CSABase::initKinematicName() {

}

void CSABase::initSensorName() {

}

void CSABase::setupRobotInterface() {
    // add kinematics/jacobian data needed for virtual fixture

}

void CSABase::setupVFInterface() {
    // data needed for virtual fixture
    mVFStateTable = new mtsStateTable(100, "VirtualFixture");
    mVFStateTable->SetAutomaticAdvance(false);

    mRobot->AddStateTable(mVFStateTable);

    // add data below

    // start state table
    mVFStateTable->Start();
}

void CSABase::initVFController(const mtsVFBase::CONTROLLERMODE controllermode) {
    mVfController = new mtsVFController(mNumDoF, controllermode);
}

void CSABase::readStateInfo() {
    // advance statetable
    mVFStateTable->Advance();
    // update kinematics and sensor
    readKinematics();
    readSensor();
    // update virtual fixture data
    readVirtualFixture();
}

void CSABase::readKinematics() {
    // read kinematics

}
void CSABase::readSensor() {
    // read kinematics
}

void CSABase::readVirtualFixture() {

}

void CSABase::solve(vctDoubleVec & dq) {
    // set kinematics
    mVfController->SetKinematics(mMeasuredKinematics);
    mVfController->SetKinematics(mGoalKinematics);

    // set sensor
    mVfController->SetSensor(mMeasuredSensor);
    mVfController->SetSensor(mGoalSensor);

    // set vf
    setVFData();
    // TODO: why statetable connot be found??
//    mVfController->UpdateOptimizer(mRobot->StateTable.GetAveragePeriod());

    // solve
    mVfController->Solve(dq);
}

void CSABase::setVFData() {

}