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

#include <sawConstraintController/mtsVFJointPos.h>

CMN_IMPLEMENT_SERVICES(mtsVFJointPosition);

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFJointPosition::FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    // Standard VF for joint incremental position control
    // min || C(dq + q) - d || => min || C(dq) - (d - Cq) ||
    // A(dq + q) >= b => A(dq) >= b - Aq
    // E(dq + q) = f => E(dq) = f - Eq

    // First check if we have all dependencies met
    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Joint Pos VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Joint Pos VF given improper input");
    }

    //Current joint position
    vctDoubleVec CurrQ;
    Kinematics.at(0)->JointState->GetPosition(CurrQ);

    double Scale = 1.0;
    if(mode == JVEL)
    {
        Scale = TickTime;
    }

    //A(dq + q) >= b
    //A*Scale*dq >= Scale*(b - A*q)
    ObjectiveMatrixRef.Assign(Data->ObjectiveMatrix*Scale*Data->Importance);
    ObjectiveVectorRef.Assign((Data->ObjectiveVector - Data->ObjectiveMatrix*CurrQ)*Scale*Data->Importance);
    IneqConstraintMatrixRef.Assign(Data->IneqConstraintMatrix*Scale);
    IneqConstraintVectorRef.Assign((Data->IneqConstraintVector - Data->IneqConstraintMatrix*CurrQ)*Scale);
    EqConstraintMatrixRef.Assign(Data->EqConstraintMatrix*Scale);
    EqConstraintVectorRef.Assign((Data->EqConstraintVector - Data->EqConstraintMatrix*CurrQ)*Scale);

    for(size_t i = 0; i < Data->NumSlacks; i++)
    {
        ObjectiveMatrixSlackRef.Column(i).SetAll(Data->SlackCosts[i]);
        IneqConstraintMatrixRef[i][i] = 1;
    }
}

void mtsVFJointPosition::ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    if(mode == JVEL)
    {
        //TickTime*A*dq >= TickTime*b
        ObjectiveMatrixRef.Assign(ObjectiveMatrixRef*TickTime);
        ObjectiveVectorRef.Assign(ObjectiveVectorRef*TickTime);
        IneqConstraintMatrixRef.Assign(IneqConstraintMatrixRef*TickTime);
        IneqConstraintVectorRef.Assign(IneqConstraintVectorRef*TickTime);
        EqConstraintMatrixRef.Assign(EqConstraintMatrixRef*TickTime);
        EqConstraintVectorRef.Assign(EqConstraintVectorRef*TickTime);
    }
}
