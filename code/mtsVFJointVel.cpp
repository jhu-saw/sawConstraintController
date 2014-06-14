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

#include <sawConstraintController/mtsVFJointVel.h>

CMN_IMPLEMENT_SERVICES(mtsVFJointVelocity);

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFJointVelocity::FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    // Standard VF for joint velocity control
    // min || C(dq) - d ||
    // A(dq) >= b
    // E(dq) = f
    switch(mode)
    {

        case JPOS:
        {
            //A/TickTime*dq >= b / TickTime
            ObjectiveMatrixRef.Assign(Data->ObjectiveMatrix / TickTime);
            ObjectiveVectorRef.Assign(Data->ObjectiveVector / TickTime);
            IneqConstraintMatrixRef.Assign(Data->IneqConstraintMatrix / TickTime);
            IneqConstraintVectorRef.Assign(Data->IneqConstraintVector / TickTime);
            EqConstraintMatrixRef.Assign(Data->EqConstraintMatrix / TickTime);
            EqConstraintVectorRef.Assign(Data->EqConstraintVector / TickTime);
            break;
        }
        case JVEL:
        {
            //A*dq >= b
            ObjectiveMatrixRef.Assign(Data->ObjectiveMatrix);
            ObjectiveVectorRef.Assign(Data->ObjectiveVector);
            IneqConstraintMatrixRef.Assign(Data->IneqConstraintMatrix);
            IneqConstraintVectorRef.Assign(Data->IneqConstraintVector);
            EqConstraintMatrixRef.Assign(Data->EqConstraintMatrix);
            EqConstraintVectorRef.Assign(Data->EqConstraintVector);
            break;
        }
        default:
        {
            CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: JointVel VF given improper mode" << std::endl;
            cmnThrow("FillInTableauRefs: JointVel VF given improper mode");
        }
    }

    for(size_t i = 0; i < Data->NumSlacks; i++)
    {
        ObjectiveMatrixSlackRef.Column(i).SetAll(Data->SlackCosts[i]);
        IneqConstraintMatrixRef[i][i] = 1;
    }
}

void mtsVFJointVelocity::ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    if(mode == JPOS)
    {
        //A*TickTime*dq >= b * TickTime
        ObjectiveMatrixRef.Assign(ObjectiveMatrixRef * TickTime);
        ObjectiveVectorRef.Assign(ObjectiveVectorRef * TickTime);
        IneqConstraintMatrixRef.Assign(IneqConstraintMatrixRef * TickTime);
        IneqConstraintVectorRef.Assign(IneqConstraintVectorRef * TickTime);
        EqConstraintMatrixRef.Assign(EqConstraintMatrixRef * TickTime);
        EqConstraintVectorRef.Assign(EqConstraintVectorRef * TickTime);
    }
}
