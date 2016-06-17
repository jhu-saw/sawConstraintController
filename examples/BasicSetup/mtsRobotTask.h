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

#include <sawConstraintController/prmJointState.h>
#include <sawConstraintController/mtsVFController.h>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

class CISST_EXPORT mtsRobotTask: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

protected:    

    //VF Controller
    mtsVFController Controller;        

    //Contains controller output
    vctDoubleVec ControllerOutput;

    prmKinematicsState CurrentKinematics, DesiredKinematics;

    prmJointState CurrentJointState; 

    vctDoubleMat Jacobian;    

    nmrConstraintOptimizer::STATUS OptimizerStatus;

public:

    mtsRobotTask(const std::string & taskName, double period): mtsTaskPeriodic(taskName, period, false, 1000) {
        
    }

    ~mtsRobotTask(void) {}

    void Setup();

    void Run();

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotTask);

#endif // _mtsRobotTask_h
