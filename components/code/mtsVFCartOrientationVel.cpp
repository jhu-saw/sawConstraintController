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

CMN_IMPLEMENT_SERVICES(mtsVFCartesianOrientation)

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
    mtsVFDataCartesian * CartData = reinterpret_cast<mtsVFDataCartesian*>(Data);

    //m = R*v is the frame rotation multiplied by the offset vector input
    m.Assign(Kinematics.at(0)->Frame.Rotation()*CartData->OffsetVector);
    m *= Data->Importance;   

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

    double Scale = 1.0;
    if(mode == JVEL)
    {
        Scale = TickTime;
    }

    // || [0 | sk(d)*sk(m)]* Scale * Jac * dq + mxd ||
    ObjectiveMatrixRef.Assign((IdentitySkewMatrix*Scale*Data->Importance)*Kinematics.at(0)->Jacobian);
    ObjectiveVectorRef.Assign((m*Scale*Data->Importance)%Data->ObjectiveVector);

    for(size_t i = 0; i < Data->NumSlacks; i++)
    {
        ObjectiveMatrixSlackRef.Column(i).SetAll(Data->SlackCosts(i));
        IneqConstraintMatrixRef(i,i) = 1;
    }
}

void mtsVFCartesianOrientation::ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    double Scale = 1.0;
    if(mode == JVEL)
    {
        Scale = TickTime;
    }
    ObjectiveMatrixRef.Assign((ObjectiveMatrixRef*Scale)*Kinematics.at(0)->Jacobian);
    ObjectiveVectorRef.Assign(ObjectiveVectorRef*Scale);
    IneqConstraintMatrixRef.Assign((IneqConstraintMatrixRef*Scale)*Kinematics.at(0)->Jacobian);
    IneqConstraintVectorRef.Assign(IneqConstraintVectorRef*Scale);
    EqConstraintMatrixRef.Assign((EqConstraintMatrixRef*Scale)*Kinematics.at(0)->Jacobian);
    EqConstraintVectorRef.Assign(EqConstraintVectorRef*Scale);

}

void mtsVFCartesianOrientation::AssignRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime, const vctDoubleVec & DOFSelections, vctDoubleMat & C, vctDoubleVec & d, vctDoubleMat & A, vctDoubleVec & b, vctDoubleMat & E, vctDoubleVec & f)
{
    double Scale = 1.0;
    if(mode == JVEL)
    {
        Scale = TickTime;
    }

    vctDoubleMat JacMat;
    JacMat.SetSize(DOFSelections.size(),C.cols());
    for(size_t i = 0; i < DOFSelections.size(); i++)
    {
        JacMat.Row(i).Assign(Kinematics.at(0)->Jacobian.Row(DOFSelections.at(i)));
    }

    ObjectiveMatrixRef.Assign((C*Scale)*JacMat);
    ObjectiveVectorRef.Assign(d*Scale);
    IneqConstraintMatrixRef.Assign((A*Scale)*JacMat);
    IneqConstraintVectorRef.Assign(b*Scale);
    EqConstraintMatrixRef.Assign((E*Scale)*JacMat);
    EqConstraintVectorRef.Assign(f*Scale);
}
