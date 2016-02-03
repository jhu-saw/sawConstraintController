/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 Author(s):  Paul Wilkening
 Created on: 2014

 (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsRobotTask_h
#define _mtsRobotTask_h

#include <cisstMultiTask/mtsTaskPeriodic.h>

#include "mtsSimpleVFController.h"

#include <sawConstraintController/prmJointState.h>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

class CISST_EXPORT mtsRobotTask: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

protected:

    enum TASKMODE {JPOS = 1, JVEL = 2};

    //VF Controller
    mtsSimpleVFController CO_Controller;    

    //Joint position and velocities
    prmJointState JointState;

    //Contains controller output
    vctDoubleVec ControllerOutput;

    nmrConstraintOptimizer::STATUS OptimizerStatus;

    //Current mode of task (how we should treat controllerOutput)
    //The possible values of MODE refer to:
    //1. Converting controllerOutput to an incremental joint position
    //2. Converting controllerOutput to a joint velocity
    //3. Converting controllerOutput to an incremental cartesian position
    //4. Converting controllerOutput to a cartesian velocity
    TASKMODE TaskMode;

public:

    mtsRobotTask(const std::string & taskName, double period): mtsTaskPeriodic(taskName, period, false, 1000) {
        // These communication methods are the same for all tasks
        InitializeInterfaces();
    }

    ~mtsRobotTask(void) {}

    // Methods needed for periodic tasks
    // (implementations of all but run can be empty if not needed)
    virtual void Run(void);

    // Set up methods that pass VFs, sensors, kins, mode, etc
    bool InitializeInterfaces(void);

    // Update local kins, sensors
    virtual bool UpdateRobotStateData(void) = 0;

    // Safety check on the CO output
    virtual bool ValidMotion(const vctDoubleVec & ControllerOutput) = 0;

    // Robot interface methods
    virtual bool EStopOn() = 0;
    virtual bool CheckWithinLimits() = 0;
    virtual bool DisableMotors() = 0;
    virtual bool StopMotion() = 0;
    virtual bool JointVelocityMove(const vctDoubleVec & dq) = 0;
    virtual bool JointPositionMove(const vctDoubleVec & dq) = 0;
    virtual bool CartesianVelocityMove(const vctDoubleVec & dx) = 0;
    virtual bool CartesianPositionMove(const vctDoubleVec & dx) = 0;
    virtual bool ComputeJointPosition(vctDoubleVec & qPos) = 0;
    virtual bool ComputeJointVelocity(vctDoubleVec & qVel) = 0;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotTask);

#endif // _mtsRobotTask_h
