/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Max Zhaoshuo Li
  Created on: 2019

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawConstraintController/mtsVFLimitsConstraint.h>
#include <sawConstraintController/mtsVFDataJointLimits.h>

CMN_IMPLEMENT_SERVICES(mtsVFLimitsConstraint)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFLimitsConstraint::FillInTableauRefs(const CONTROLLERMODE mode,
                                              const double CMN_UNUSED(tickTime))
{
    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "mtsVFLimitsConstraint FillInTableauRefs: kinematics needed" << std::endl;
        cmnThrow("mtsVFLimitsConstraint FillInTableauRefs: kinematics needed");
    }

    vctDoubleVec jointPositions;
    Kinematics.at(0)->JointState->GetPosition(jointPositions);

    mtsVFDataJointLimits * limitData = reinterpret_cast<mtsVFDataJointLimits*>(Data);
    size_t numLimits = limitData->LowerLimits.size();

    IneqConstraintMatrixRef.SetAll(0.0);

    if((mode == JPOS || mode == CARTPOS) && limitData->AbsoluteLimit) {
        for(size_t i = 0; i < numLimits; i++) {
            IneqConstraintMatrixRef.at(i,i) = 1.0;
            IneqConstraintMatrixRef.at(i+numLimits,i) = -1.0;
            IneqConstraintVectorRef.at(i) = limitData->LowerLimits.at(i) - jointPositions.at(i);
            IneqConstraintVectorRef.at(i+numLimits) = -limitData->UpperLimits.at(i) + jointPositions.at(i);
        }
    } else if(mode == JVEL || mode == CARTVEL || !limitData->AbsoluteLimit) {
        for(size_t i = 0; i < numLimits; i++) {
            IneqConstraintMatrixRef.at(i,i) = 1.0;
            IneqConstraintMatrixRef.at(i+numLimits,i) = -1.0;
            IneqConstraintVectorRef.at(i) = limitData->LowerLimits.at(i);
            IneqConstraintVectorRef.at(i+numLimits) = -limitData->UpperLimits.at(i);
        }
    }
}

void mtsVFLimitsConstraint::ConvertRefs(const CONTROLLERMODE CMN_UNUSED(mode),
                                        const double CMN_UNUSED(tickTime))
{

}
