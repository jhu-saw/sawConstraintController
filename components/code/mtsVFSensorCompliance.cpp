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
    // min || J(q)*dq - gain*overallGain*sensorValues ||

    // Check if we have a jacobian to use
    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error: Sensor Compliance VF is missing kinematics dependencies" << std::endl;
        cmnThrow("Error: Sensor Compliance VF is missing kinematics dependencies");
    }    

    mtsVFDataSensorCompliance * SensorComplianceData = (mtsVFDataSensorCompliance *)(Data);
    vctDoubleMat * JacP = &(Kinematics.at(0)->Jacobian);
    vctDynamicVector<double> * SensorValues = &(Sensors.at(0)->Values);
    vctDynamicVector<double> overallGain = Sensors.at(1)->Values;        

    //choose only selected sensor values (indices stored in SensorSelections)
    vctDynamicVector<double> UsedValues(SensorComplianceData->Gain.cols());
    for (size_t s=0; s < SensorComplianceData->SensorSelections.size(); s++)
    {
        UsedValues[s] = (overallGain[0]/100.0)*((*SensorValues)[SensorComplianceData->SensorSelections[s]]);
    }

    // Check if we have all dependencies met
    if(Sensors.size() < 2 || SensorComplianceData->Gain.rows() != JacP->rows() || SensorComplianceData->Gain.cols() != UsedValues.size())
    {        
        CMN_LOG_CLASS_RUN_ERROR << "Error: Sensor Compliance VF is missing sensor dependencies" << std::endl;
        cmnThrow("Error: Sensor Compliance VF is missing sensor dependencies");
    }    

    //set the reference to the right hand side of the above equation (gain*Force)
    ObjectiveVectorRef.Assign(SensorComplianceData->Gain * UsedValues);

    //now set reference to the left hand side of the above equation, the jacobian (only rows indicated in DOFSelections)
    for(size_t r = 0; r < SensorComplianceData->DOFSelections.size(); r++)
    {
        for (size_t c = 0; c < JacP->cols(); c++)
        {
            if(r == c)
            {
                ObjectiveMatrixRef[r][c] = (*JacP)[r][c];
            }
        }
    }

    // ConvertRefs(mode,TickTime);
}
