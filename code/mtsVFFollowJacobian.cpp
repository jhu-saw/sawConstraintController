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

#include <sawConstraintController/mtsVFFollow.h>
#include <stdlib.h>
#include <cisstNumerical/nmrInverse.h>

CMN_IMPLEMENT_SERVICES(mtsVFFollow)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFFollow::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    // fill in refs
    // min || J*dq - [v_T ; v_R] ||
    // v_T = p_des - R_des * R^(-1)_cur * p_cur
    // v_R = sk(A)^(-1) * A - sk(A)^(-1) * R_des * R^(-1)_cur * A
    // J is the da vinci's jacobian, p_des is the desired frame's translation,
    // R_des is the desired frame's rotation, R^(-1)_cur is the inverse of the current frame's rotation,
    // p_cur is the current frame's translation, A is an arbitrary vector, sk(A)^(-1) is the inverse of the skew matrix of A

    //check desired frame, current frame dependencies
    if(Kinematics.size() < 2)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Follow VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Follow VF given improper input");
    }

    // set arbitrary vector
    A.SetSize(3);
    A.Assign(rand() % 100 + 1, rand() % 100 + 1, rand() % 100 + 1);
    A = A.NormalizedSelf();

    // set inverse skew matrix of A
    skewAInverse.SetSize(3,3);
    skewAInverse.SetAll(0.0);
    skewAInverse(0,1) = -A[2];
    skewAInverse(0,2) = A[1];
    skewAInverse(1,0) = A[2];
    skewAInverse(1,2) = -A[0];
    skewAInverse(2,0) = -A[1];
    skewAInverse(2,1) = A[0];
    nmrInverse(skewAInverse);

    // pointers to kinematics
    CurrentKinematics = Kinematics.at(0);
    DesiredKinematics = Kinematics.at(1);

    // current kinematics gives us current frame
    CurrentFrame = CurrentKinematics->Frame;

    // desired kinematics gives us desired frame
    DesiredFrame = DesiredKinematics->Frame;

    // Jacobian
    std::cout << "Obj Mat Ref " << ObjectiveMatrixRef.sizes() << std::endl;
    std::cout << "Jac " << CurrentKinematics->Jacobian.sizes() << std::endl;
    ObjectiveMatrixRef.Assign(CurrentKinematics->Jacobian);

    // p_des - R_des * R^(-1)_cur * p_cur
    TranslationObjectiveVector.SetSize(3);
    TranslationObjectiveVector.Assign(DesiredFrame.Translation() - DesiredFrame.Rotation() * CurrentFrame.Rotation().Inverse() * CurrentFrame.Translation());
    // sk(A)^(-1) * A - sk(A)^(-1) * R_des * R^(-1)_cur * A
    RotationObjectiveVector.SetSize(3);
    RotationObjectiveVector.Assign(skewAInverse * A - skewAInverse * vctDoubleMat(DesiredFrame.Rotation()) * vctDoubleMat(CurrentFrame.Rotation().Inverse()) * A);

    for(size_t i = 0; i < 3; i++)
    {
        ObjectiveVectorRef[i] = TranslationObjectiveVector[i];
    }
    for(size_t i = 3; i < 7; i++)
    {
        ObjectiveVectorRef[i] = RotationObjectiveVector[i-3];
    }

    // make conversion, if necessary
    ConvertRefs(mode,TickTime);

    std::cout << "Vec Obj \n" << ObjectiveVectorRef << std::endl;
    std::cout << "Mat Obj \n" << ObjectiveMatrixRef << std::endl;

}