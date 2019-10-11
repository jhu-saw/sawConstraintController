//
// Created by max on 2019-10-10.
//

#ifndef SAWCONSTRAINTCONTROLLER_CSABASE_H
#define SAWCONSTRAINTCONTROLLER_CSABASE_H

#include <cisstCommon/cmnGenericObject.h>

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsStateTable.h>

#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <sawConstraintController/prmSensorState.h>

class CSABase: cmnGenericObject {
public:
    CSABase(const int &numDoF, const int &numJoints, mtsComponent *component);
    virtual ~CSABase(){};
    
    // initialize
    void init(void);
    virtual void setupRobotInterface(); // setup robot required/provided interfaces
    virtual void setupVFInterface(); // setup vf required/provided interfaces
    virtual void initParameters(); // to be overridden, other initialization needed for base type
    void initVFController(const mtsVFBase::CONTROLLERMODE controllermode);
    virtual void initKinematicName();
    virtual void initSensorName();

    // read
    void readStateInfo();
    virtual void readKinematics(void); // TODO: should be merged into set??
    virtual void readSensor(void);
    virtual void readVirtualFixture(void);

    // solve
    void solve(vctDoubleVec & dq);
    virtual void setVFData();

private:
    mtsVFController  *mVfController;  // Constraint controller
    mtsStateTable* mVFStateTable; // state table for virtual fixture data

    int mNumDoF;
    int mNumJoints;
    mtsComponent *mRobot;
    prmKinematicsState mMeasuredKinematics;
    prmKinematicsState mGoalKinematics;
    prmSensorState mMeasuredSensor;
    prmSensorState mGoalSensor;
};


#endif //SAWCONSTRAINTCONTROLLER_CSABASE_H
