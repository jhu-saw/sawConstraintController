/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening
  Created on:
 
  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.
 
 --- begin cisst license - do not edit ---
 
 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.
 
 --- end cisst license ---
 */

#ifndef _mtsVFControllerTask_h
#define _mtsVFControllerTask_h

#include <cisstVector/vctTypes.h>
#include <cisstCommon/cmnPath.h>
#include <sawConstraintController/prmSensorState.h>
#include "prmSimpleRobotKinematicsState.h"
#include <cisstNumerical/nmrConstraintOptimizer.h>
#include <sawConstraintController/prmForceOffsetState.h>
#include <sawConstraintController/mtsVFController.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include "mtsRobotTask.h"

enum {NB_Joints = 2};

class mtsVFControllerTask : public mtsRobotTask
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

private:

	// Kinematics, joint state, and sensors
    prmSimpleRobotKinematicsState EEKin;
    prmSensorState pedal;   
    int tickNum;

public:

    mtsVFControllerTask(const std::string & taskName, double period);
    ~mtsVFControllerTask(void) {}

    void Configure(const std::string & CMN_UNUSED(filename) = "") {}
    void Startup(void) {}
    void Cleanup(void) {}

    bool InitializeInterfaces(void);
    bool UpdateRobotStateData(void);
    bool ValidMotion(const vctDoubleVec & dq) 
    {
        vctDoubleVec ans(2);
        ans(0) = cos(2*cmnPI*tickNum/100);
        ans(1) = sin(2*cmnPI*tickNum/100);
        // std::cout << "Diff: " << (ans-dq).Norm() << std::endl;
        //std::cout << dq << std::endl;
        if((ans-dq).Norm() < 0.1)
        {
            std::cout << "CO output matches expected results. Diff = " << (ans-dq).Norm() << std::endl;
            return true;
        }
        else
        {
            std::cout << "CO output doesn't match expected results. Diff = " << (ans-dq).Norm() << std::endl;
            return false;
        }
    }

    // These are implementations of robot-specific virtual methods
    bool CheckWithinLimits(void) {return true;}
    bool EStopOn(void) {return false;}
    bool DisableMotors(void) {return true;}
    bool StopMotion(void) {return true;}
    bool JointVelocityMove(const vctDoubleVec &) {return true;}
    bool JointPositionMove(const vctDoubleVec &) {return true;}
    bool CartesianVelocityMove(const vctDoubleVec &) {return true;}
    bool CartesianPositionMove(const vctDoubleVec &) {return true;}
    bool ComputeJointPosition(vctDoubleVec &) {return true;}
    bool ComputeJointVelocity(vctDoubleVec &) {return true;}

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFControllerTask)

#endif
