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

#include <sawConstraintController/mtsVFSensorCompliance.h>

CMN_IMPLEMENT_SERVICES(mtsVFSensorCompliance);

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFSensorCompliance::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    // fill in refs
    // min || J(q)*dq - gain*sensorValues ||

    // Check if we have a jacobian to use
    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error: Sensor Compliance VF is missing kinematics dependencies" << std::endl;
        cmnThrow("Error: Sensor Compliance VF is missing kinematics dependencies");
    }    

    if(Sensors.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error: Sensor Compliance VF is missing sensor dependencies" << std::endl;
        cmnThrow("Error: Sensor Compliance VF is missing sensor dependencies");
    }

    mtsVFDataSensorCompliance * gainData = (mtsVFDataSensorCompliance *)(Data);
    vctDoubleMat Jacobian = Kinematics.at(0)->Jacobian;
    vctDoubleVec SensorValues = Sensors.at(0)->Values;

    //set the reference to the right hand side of the above equation (gain*Force)
    ObjectiveMatrixRef.Assign(Jacobian);
    ObjectiveVectorRef.ElementwiseProductOf(gainData->Gain, SensorValues);

    ConvertRefs(mode,TickTime);
}

void mtsVFSensorCompliance::ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    if (mode == mtsVFBase::CONTROLLERMODE::JVEL){
        // min || J(q)*t*v - gain*sensorValues ||
        ObjectiveMatrixRef.Multiply(TickTime);
    }
}

