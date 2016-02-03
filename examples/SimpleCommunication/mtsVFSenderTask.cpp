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

#include "mtsVFSenderTask.h"
#include <cisstMultiTask/mtsInterfaceRequired.h>

// #include <stdio.h>

CMN_IMPLEMENT_SERVICES(mtsVFSenderTask);

mtsVFSenderTask::mtsVFSenderTask(const std::string & taskName, double period) : mtsTaskPeriodic(taskName, period, false, 1000)
{
	//Creates VF data objects
    vctDoubleMat gain(2,2);
    gain.SetAll(0);
    for(size_t i = 0; i < 2; i++)
    {
        gain[i][i] = 1;
    }
	forceVF = mtsVFDataSensorCompliance("ForceVF","EEKin","Force","Pedal",gain);
    vctDynamicVector<size_t> jointIndices(NUM_JOINTS);
    //TODO handle DOFSelections in other VFs (probably just stick it in a function)
    for(size_t i = 0; i < NUM_JOINTS; i++)
    {
        jointIndices[i] = i;
    }
    forceVF.DOFSelections = jointIndices;
    vctDynamicVector<size_t> sensorIndices(NB_FT);
    for(size_t i = 0; i < NB_FT; i++)
    {
        sensorIndices[i] = i;
    }
    forceVF.SensorSelections.SetSize(NB_FT);
    forceVF.SensorSelections.Assign(sensorIndices);
    forceVF.Active = true;

    //Sensors
    forceSensorState = prmSensorState("Force");
    forceSensorState.Values.SetSize(2);
    forceSensorState.Values.SetAll(0);
    tickNum = 1;
}

void mtsVFSenderTask::Configure(const std::string & CMN_UNUSED(filename))
{
    //link the functions defined in the header with strings that match those of the run loop example
    mtsInterfaceRequired * required = this->AddInterfaceRequired("RequiresVF");
    if (!required) {
        CMN_LOG_CLASS_RUN_ERROR << "Creating interface failed" << std::endl;
    }
    if (!(required->AddFunction("AddVFSensorCompliance", this->AddVFSensorCompliance))) {
        CMN_LOG_CLASS_RUN_ERROR << "Adding AddVFSensorCompliance failed" << std::endl;
    }
    if (!(required->AddFunction("SetSensor", this->SetSensor))) {
        CMN_LOG_CLASS_RUN_ERROR << "Adding SetSensor failed" << std::endl;
    }
}

void mtsVFSenderTask::Run()
{
    tickNum++;
	this->ProcessQueuedEvents();
	this->ProcessQueuedCommands();    

    UpdateRobotStateData();

    SetSensor(forceSensorState);

    AddVFSensorCompliance(forceVF);

}

void mtsVFSenderTask::UpdateRobotStateData()
{
    forceSensorState.Values[0] = cos(2*cmnPI*tickNum/100);
    forceSensorState.Values[1] = sin(2*cmnPI*tickNum/100);
    SetSensor(forceSensorState);
}
