/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-05-14

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _simpleRobot_h
#define _simpleRobot_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFFollow.h>
#include <sawConstraintController/mtsVFPlane.h>
#include <sawConstraintController/mtsVFLimitsConstraint.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/mtsVFDataPlane.h>
#include <sawConstraintController/mtsVFDataJointLimits.h>

class simpleRobot: public mtsTaskPeriodic {

    // used to control the log level, "Run Error" by default
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);
protected:
    // internal method to configure this component
    void init();
    void setupRobot();
    void setupVF();
    nmrConstraintOptimizer::STATUS solve(vctDoubleVec & dq);

    // robot specific variables
    void forwardKinematics(vctDoubleVec& jointPosition);
    vctDoubleVec mJointPosition;
    vctDoubleMat mJacobian;
    vctFrm4x4 mCartesianPosition;
    int mNumDof;
    int mNumJoints;

    // constraint controller
    mtsVFController *mController;
    prmKinematicsState mMeasuredKinematics; // follow crtk convention
    prmKinematicsState mGoalKinematics;
    prmStateJoint mMeasuredJoint;
    mtsVFDataBase mTeleopObjective; // No additional data needed, therefore using mtsVFBase
    mtsVFDataPlane mPlaneConstraint;
    mtsVFDataJointLimits mJointLimitsConstraint;
    void updateKinematics();

    // teleop command
    void servoCartesianRelative(const mtsFrm4x4 & newGoal);


public:
    // provide a name for the task and define the frequency (time
    // interval between calls to the periodic Run).  Also used to
    // populate the interface(s)
    simpleRobot(const std::string & componentName, const double periodInSeconds);

    ~simpleRobot() {};

    // all four methods are pure virtual in mtsTask
    void Run();        // performed periodically
};

CMN_DECLARE_SERVICES_INSTANTIATION(simpleRobot);

#endif // _simpleRobot_h