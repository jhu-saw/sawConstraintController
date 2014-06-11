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

#include <sawConstraintController/mtsVFCartVel.h>
#include <sawConstraintController/mtsVFDataCartesianTranslation.h>

CMN_IMPLEMENT_SERVICES(mtsVFCartesianTranslation)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFCartesianTranslation::FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    // Standard VF for cartesian velocity control
    // min || C * (F* * w - d) || => || C[I | sk(-w)] dx - C(w + d) ||
    // A * (F* * w - b) >= 0 => A[I | sk(-w)] dx >= A(w + b)
    // E * (F* * w - f) = 0 => E[I | sk(-w)] dx = E(w + f)

    // First check if we have all dependencies met
    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Cart Trans VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Cart Trans VF given improper input");
    }

    mtsVFDataCartesian * CartData = (mtsVFDataCartesian *)Data;

    //w is the transformed vector input
    w.Assign((Kinematics.at(0)->Frame)*CartData->OffsetVector);

    //Fill in identity part of [I | sk(-w)]
    IdentitySkewMatrix.SetSize(3,6);
    IdentitySkewMatrix.SetAll(0.0);
    for(size_t i = 0; i < 3; i++)
    {        
        IdentitySkewMatrix(i,i) = 1;
    }

    //Fill in skew part using w (skew of -w)
    IdentitySkewMatrix(0,1) = w(2);
    IdentitySkewMatrix(0,2) = -w(1);
    IdentitySkewMatrix(1,0) = -w(2);
    IdentitySkewMatrix(1,2) = w(0);
    IdentitySkewMatrix(2,0) = w(1);
    IdentitySkewMatrix(2,1) = -w(0);

    switch(mode)
    {

        case JPOS:
        {
            //A[I;sk(-w)]*Jac*dq >= A*(w+b)
            ObjectiveMatrixRef.Assign(Data->ObjectiveMatrix*IdentitySkewMatrix*Kinematics.at(0)->Jacobian);
            ObjectiveVectorRef.Assign(Data->ObjectiveMatrix * (w + Data->ObjectiveVector));
            IneqConstraintMatrixRef.Assign(Data->IneqConstraintMatrix*IdentitySkewMatrix*Kinematics.at(0)->Jacobian);
            IneqConstraintVectorRef.Assign(Data->IneqConstraintMatrix * (w + Data->IneqConstraintVector));
            EqConstraintMatrixRef.Assign(Data->EqConstraintMatrix*IdentitySkewMatrix*Kinematics.at(0)->Jacobian);
            EqConstraintVectorRef.Assign(Data->EqConstraintMatrix * (w + Data->EqConstraintVector));
            break;
        }
        case JVEL:
        {                        
            //1/TickTime*A[I;sk(-w)]*Jac*dq >= 1/TickTime*A*(w+b)
            ObjectiveMatrixRef.Assign(1/TickTime*Data->ObjectiveMatrix*IdentitySkewMatrix*Kinematics.at(0)->Jacobian);
            ObjectiveVectorRef.Assign(1/TickTime*Data->ObjectiveMatrix * (w + Data->ObjectiveVector));
            IneqConstraintMatrixRef.Assign(1/TickTime*Data->IneqConstraintMatrix*IdentitySkewMatrix*Kinematics.at(0)->Jacobian);
            IneqConstraintVectorRef.Assign(1/TickTime*Data->IneqConstraintMatrix * (w + Data->IneqConstraintVector));
            EqConstraintMatrixRef.Assign(1/TickTime*Data->EqConstraintMatrix*IdentitySkewMatrix*Kinematics.at(0)->Jacobian);
            EqConstraintVectorRef.Assign(1/TickTime*Data->EqConstraintMatrix * (w + Data->EqConstraintVector));
            break;
        }
        default:
        {
            CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Cart Trans VF given improper mode" << std::endl;
            cmnThrow("FillInTableauRefs: Cart Trans VF given improper mode");
        }
    }

    for(size_t i = 0; i < Data->NumSlacks; i++)
    {
        ObjectiveMatrixSlackRef.Column(i).SetAll(Data->SlackCosts[i]);
        IneqConstraintMatrixRef(i,i) = 1;
    }
}

void mtsVFCartesianTranslation::ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    switch(mode)
    {
        case JPOS:
        {
            ObjectiveMatrixRef.Assign(ObjectiveMatrixRef*Kinematics.at(0)->Jacobian);
            IneqConstraintMatrixRef.Assign(IneqConstraintMatrixRef*Kinematics.at(0)->Jacobian);
            EqConstraintMatrixRef.Assign(EqConstraintMatrixRef*Kinematics.at(0)->Jacobian);
            break;
        }
        case JVEL:
        {
            ObjectiveMatrixRef.Assign(1/TickTime*ObjectiveMatrixRef*Kinematics.at(0)->Jacobian);
            ObjectiveVectorRef.Assign(1/TickTime*ObjectiveVectorRef);
            IneqConstraintMatrixRef.Assign(1/TickTime*IneqConstraintMatrixRef*Kinematics.at(0)->Jacobian);
            IneqConstraintVectorRef.Assign(1/TickTime*IneqConstraintVectorRef);
            EqConstraintMatrixRef.Assign(1/TickTime*EqConstraintMatrixRef*Kinematics.at(0)->Jacobian);
            EqConstraintVectorRef.Assign(1/TickTime*EqConstraintVectorRef);
            break;
        }
        default:
        {
            CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Cart Trans VF given improper mode" << std::endl;
            cmnThrow("FillInTableauRefs: Cart Trans VF given improper mode");
        }
    }
}
