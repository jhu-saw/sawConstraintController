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

#include "mtsRobotTask.h"
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES(mtsRobotTask);

bool mtsRobotTask::InitializeInterfaces(void)
{
    mtsVFController * CO_Controller_Base = dynamic_cast<mtsVFController *>(&CO_Controller);
    mtsInterfaceProvided * provided = this->AddInterfaceProvided("ProvidesVF");
    if (!provided) {
        CMN_LOG_CLASS_RUN_ERROR << "Creating interface failed " << std::endl;
        return false;
    }
    if (!(provided->AddCommandWrite(&mtsVFController::AddVFSensorCompliance, CO_Controller_Base, "AddVFSensorCompliance"))) {
        CMN_LOG_CLASS_RUN_ERROR << "SetVFSensorCompliance failed" << std::endl;
        return false;
    }    
    if (!(provided->AddCommandWrite(&mtsVFController::SetSensor, CO_Controller_Base, "SetSensor"))) {
        CMN_LOG_CLASS_RUN_ERROR << "SetSensor failed" << std::endl;
        return false;
    }

    return true;
}

void mtsRobotTask::Run()
{
    //Receive VFs, sensors, kinematics, new mode
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    //Update local VFs, kins, sensors
    UpdateRobotStateData();

    //Check if the emergency stop is pressed
    if (EStopOn() || !CheckWithinLimits())
    {
        //Halt all motion
        StopMotion();
    }
    else
    {
        try
        {
            // Go through the VF list, update state data pointers, assign tableau references, and fill in the references
            CO_Controller.UpdateOptimizer(this->GetPeriodicity());

            // Compute a new motion and check the solve return code
            OptimizerStatus = CO_Controller.Solve(ControllerOutput);

            //Check if the CO output is a valid movement of the robot
            bool valid_velocity = ValidMotion(ControllerOutput);

            if(valid_velocity && OptimizerStatus == nmrConstraintOptimizer::NMR_OK)
            {
                switch(CO_Controller.ControllerMode)
                {
                    case mtsVFBase::JPOS:
                        switch(TaskMode)
                        {
                            //Joint velocity mode
                            case JVEL:
                                JointVelocityMove(ControllerOutput/this->GetPeriodicity());
                                break;
                            //Incremental joint position mode
                            case JPOS:
                                JointPositionMove(ControllerOutput + JointState.JointPosition);
                                break;
                            default:
                                CMN_LOG_CLASS_RUN_ERROR << "Invalid task mode" << std::endl;
                                cmnThrow("Invalid task mode");
                        }
                        break;
                    case mtsVFBase::JVEL:
                        switch(TaskMode)
                        {
                            //Joint velocity mode
                            case JVEL:
                                JointVelocityMove(ControllerOutput);
                                break;
                            //Incremental joint position mode
                            case JPOS:
                                JointPositionMove(this->GetPeriodicity()*ControllerOutput + JointState.JointPosition);
                                break;
                            default:
                                CMN_LOG_CLASS_RUN_ERROR << "Invalid task mode" << std::endl;
                                cmnThrow("Invalid task mode");
                        }
                        break;
                    default:
                    {
                        CMN_LOG_CLASS_RUN_ERROR << "Invalid controller mode" << std::endl;
                        cmnThrow("Invalid controller mode");
                    }
                }
            }
            else
            {
                //If some error was detected, don't move the robot and return an error
                StopMotion();
                if(!valid_velocity)
                {
                    CMN_LOG_CLASS_RUN_ERROR << "Joint velocity invalid" << std::endl;
                }
                if(OptimizerStatus != nmrConstraintOptimizer::NMR_OK)
                {
                    CMN_LOG_CLASS_RUN_ERROR << "Control Optimizer returned status: " << OptimizerStatus << std::endl;
                }
            }
        }
        catch(std::exception e)
        {
            CMN_LOG_CLASS_RUN_ERROR << "Error preventing control optimizer from executing: " << e.what() << std::endl;
        }
    }
}
