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

#include <sawConstraintController/mtsVFCartOrientationVel.h>
#include <sawConstraintController/mtsVFDataCartesianTranslation.h>

CMN_IMPLEMENT_SERVICES(mtsVFCartesianOrientation);

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFCartesianOrientation::FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    // Standard VF for cartesian velocity control
    // Given a rotation R and an offset from the Frame of this rotation v (where we will define the VF)
    // We define R* as dR * R and m as R * v
    // Here, d is our objective vector
    // We aim to minimize the following expression by our selection of dx
    // min || R* * v x d || => min || [0 | sk(d)*sk(m)] * dx + m x d ||

    // First check if we have all dependencies met
    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Cart Orientation VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Cart Orientation VF given improper input");
    }

    // Convert base class of VF data to a cartesian VF data subclass to access OffsetVector
    mtsVFDataCartesian * CartData = (mtsVFDataCartesian *)Data;

    //m = R*v is the frame rotation multiplied by the offset vector input
    m.Assign(Kinematics.at(0)->Frame.Rotation()*CartData->OffsetVector);
    m *= Data->Importance;
   // old  m.Assign(Kinematics.at(0)->Frame.Rotation().ApplyTo(CartData->OffsetVector));

    //Fill in [0 | sk(d)*sk(m) ]
    IdentitySkewMatrix.SetSize(3,6);
    IdentitySkewMatrix.SetAll(0.0);
    IdentitySkewMatrix(0,0) = -Data->ObjectiveVector(2)*m(2) - Data->ObjectiveVector(1)*m(1);
    IdentitySkewMatrix(0,1) = Data->ObjectiveVector(1)*m(0);
    IdentitySkewMatrix(0,2) = Data->ObjectiveVector(2)*m(0);
    IdentitySkewMatrix(1,0) = Data->ObjectiveVector(0)*m(1);
    IdentitySkewMatrix(1,1) = -Data->ObjectiveVector(2)*m(2) - Data->ObjectiveVector(0)*m(0);
    IdentitySkewMatrix(1,2) = Data->ObjectiveVector(2)*m(1);
    IdentitySkewMatrix(2,0) = Data->ObjectiveVector(0)*m(2);
    IdentitySkewMatrix(2,1) = Data->ObjectiveVector(1)*m(2);
    IdentitySkewMatrix(2,2) = -Data->ObjectiveVector(1)*m(1) - Data->ObjectiveVector(0)*m(0);

    switch(mode)
    {

        case JPOS:
        {
            // || [0 | sk(d)*sk(m)] * Jac * dq + mxd ||
            ObjectiveMatrixRef.Assign(IdentitySkewMatrix*Kinematics.at(0)->Jacobian);
            ObjectiveVectorRef.Assign(m%Data->ObjectiveVector);
            break;
        }
        case JVEL:
        {
            // || TickTime * [0 | sk(d)*sk(m)] * Jac * dx + TickTime * mxd ||
            ObjectiveMatrixRef.Assign(TickTime*IdentitySkewMatrix*Kinematics.at(0)->Jacobian);
            ObjectiveVectorRef.Assign(TickTime*m%Data->ObjectiveVector);
            break;
        }
        default:
        {
            CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: CartVel VF given improper mode" << std::endl;
            cmnThrow("FillInTableauRefs: CartVel VF given improper mode");
        }
    }

    for(size_t i = 0; i < Data->NumSlacks; i++)
    {
        ObjectiveMatrixSlackRef.Column(i).SetAll(Data->SlackCosts(i));
        IneqConstraintMatrixRef(i,i) = 1;
    }
}

void mtsVFCartesianOrientation::ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
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
