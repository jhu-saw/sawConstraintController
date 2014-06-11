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

    vctDoubleVec * CurrQ = new vctDoubleVec();
    CurrQ->Assign(Kinematics.at(0)->JointState->JointPosition);

    switch(mode)
    {

        case JPOS:
        {
            //A(dq + q) >= b
            //A*dq >= b - A*q
            ObjectiveMatrixRef.Assign(Data->ObjectiveMatrix);
            ObjectiveVectorRef.Assign(Data->ObjectiveVector - Data->ObjectiveMatrix*(*CurrQ));
            IneqConstraintMatrixRef.Assign(Data->IneqConstraintMatrix);
            IneqConstraintVectorRef.Assign(Data->IneqConstraintVector - Data->IneqConstraintMatrix*(*CurrQ));
            EqConstraintMatrixRef.Assign(Data->EqConstraintMatrix);
            EqConstraintVectorRef.Assign(Data->EqConstraintVector - Data->EqConstraintMatrix*(*CurrQ));
            break;
        }
        case JVEL:
        {
            //1/TickTime*A*dq >= 1/TickTime*b
            ObjectiveMatrixRef.Assign(1/TickTime*Data->ObjectiveMatrix);
            ObjectiveVectorRef.Assign(1/TickTime * Data->ObjectiveVector);
            IneqConstraintMatrixRef.Assign(1/TickTime*Data->IneqConstraintMatrix);
            IneqConstraintVectorRef.Assign(1/TickTime * Data->IneqConstraintVector);
            EqConstraintMatrixRef.Assign(1/TickTime*Data->EqConstraintMatrix);
            EqConstraintVectorRef.Assign(1/TickTime*Data->EqConstraintVector);
            break;
        }
        default:
        {
            CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: JointPos VF given improper mode" << std::endl;
            cmnThrow("FillInTableauRefs: JointPos VF given improper mode");
        }
    }

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
        //1/TickTime*A*dq >= 1/TickTime*b
        ObjectiveMatrixRef.Assign(1/TickTime*ObjectiveMatrixRef);
        ObjectiveVectorRef.Assign(1/TickTime * ObjectiveVectorRef);
        IneqConstraintMatrixRef.Assign(1/TickTime*IneqConstraintMatrixRef);
        IneqConstraintVectorRef.Assign(1/TickTime * IneqConstraintVectorRef);
        EqConstraintMatrixRef.Assign(1/TickTime*EqConstraintMatrixRef);
        EqConstraintVectorRef.Assign(1/TickTime*EqConstraintVectorRef);
    }
}
