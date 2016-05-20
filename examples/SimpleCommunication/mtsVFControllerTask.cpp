/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 $Id: $
 
 Author(s):  Paul Wilkening
 Created on:
 
 (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.
 
 --- begin cisst license - do not edit ---
 
 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.
 
 --- end cisst license ---
 */

#include "mtsVFControllerTask.h"

CMN_IMPLEMENT_SERVICES(mtsVFControllerTask)

mtsVFControllerTask::mtsVFControllerTask(const std::string & taskName, double period) : mtsRobotTask(taskName, period)
{    
    mtsVFBase::DefaultKinematicsName = "EEKin";

    //CO variables
    TaskMode = JVEL;
    CO_Controller = mtsSimpleVFController(NB_Joints,mtsVFBase::JVEL);
    ControllerOutput.SetSize(NB_Joints);

	//Kinematics objects	
    JointState.JointPosition.SetSize(NB_Joints);
    JointState.JointVelocity.SetSize(NB_Joints);
    JointState.JointPosition.SetAll(0);
    JointState.JointVelocity.SetAll(0);
    EEKin = prmSimpleRobotKinematicsState("EEKin",&JointState);     
    CO_Controller.SetSimpleRobotKinematics(EEKin);
    pedal = prmSensorState("Pedal");
    pedal.Values.SetSize(1);
    CO_Controller.SetSensor(pedal);

    CO_Controller.ControllerMode = mtsVFBase::JVEL;

    tickNum = 0;    

}

bool mtsVFControllerTask::UpdateRobotStateData()
{    
    //update local sensor/kin data
    if(EEKin.UserCount > 0)
    {
        EEKin.Update();
        CO_Controller.SetSimpleRobotKinematics(EEKin);
    }
    if(pedal.UserCount > 0)
    {
        pedal.Values[0] = 100;
        CO_Controller.SetSensor(pedal);
    }

    tickNum++;

    return true;
}
