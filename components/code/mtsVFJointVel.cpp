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

    double Scale = 1.0;
    if(mode == JPOS)
    {
        Scale = 1/TickTime;
    }

    //A/TickTime*dq >= b / TickTime
    ObjectiveMatrixRef.Assign(Data->ObjectiveMatrix * Scale * Data->Importance);
    ObjectiveVectorRef.Assign(Data->ObjectiveVector * Scale * Data->Importance);
    IneqConstraintMatrixRef.Assign(Data->IneqConstraintMatrix * Scale);
    IneqConstraintVectorRef.Assign(Data->IneqConstraintVector * Scale);
    EqConstraintMatrixRef.Assign(Data->EqConstraintMatrix * Scale);
    EqConstraintVectorRef.Assign(Data->EqConstraintVector * Scale);

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
        ObjectiveMatrixRef.Assign(ObjectiveMatrixRef / TickTime);
        ObjectiveVectorRef.Assign(ObjectiveVectorRef / TickTime);
        IneqConstraintMatrixRef.Assign(IneqConstraintMatrixRef / TickTime);
        IneqConstraintVectorRef.Assign(IneqConstraintVectorRef / TickTime);
        EqConstraintMatrixRef.Assign(EqConstraintMatrixRef / TickTime);
        EqConstraintVectorRef.Assign(EqConstraintVectorRef / TickTime);
    }
}
