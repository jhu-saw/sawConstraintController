/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: $
 
  Author(s):  Paul Wilkening
  Created on:
 
  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.
 
 --- begin cisst license - do not edit ---
 
 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.
 
 --- end cisst license ---
 */

#include <cisstOSAbstraction/osaSleep.h>

#include "mtsVFControllerTask.h"
#include "mtsVFSenderTask.h"

int main(int, char **)
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    cmnLogger::SetMaskClassMatching("mts", CMN_LOG_ALLOW_ALL);

    mtsComponentManager * localManager = mtsComponentManager::GetInstance();

	//sets up variables
    mtsVFControllerTask * eyeRobotTask = new mtsVFControllerTask("EyeRobotTask", 5 * cmn_ms);
    mtsVFSenderTask * VFUserTask = new mtsVFSenderTask("VFUserTask", 5 * cmn_ms);

	//adds functions	
	eyeRobotTask->Configure();
    VFUserTask->Configure();

	//add our task to the manager
	localManager->AddComponent(eyeRobotTask);
    localManager->AddComponent(VFUserTask);

    localManager->Connect(VFUserTask->GetName(), "RequiresVF",
                          eyeRobotTask->GetName(), "ProvidesVF");

    localManager->CreateAll();

    localManager->WaitForStateAll(mtsComponentState::READY);

    localManager->StartAll();

    localManager->WaitForStateAll(mtsComponentState::ACTIVE);

	std::cout << "Running" << std::endl;

    while (true) {
        osaSleep(100.0 * cmn_ms);
    }

    localManager->KillAll();
    localManager->WaitForStateAll(mtsComponentState::FINISHED);

    cmnLogger::Kill();

    return 0;
}
