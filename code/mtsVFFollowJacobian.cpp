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

#include <sawConstraintController/mtsVFFollowJacobian.h>
#include <stdlib.h>
#include <cisstNumerical/nmrInverse.h>

CMN_IMPLEMENT_SERVICES(mtsVFFollowJacobian)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFFollowJacobian::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
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
    A.SetAll(0.0);
    A[2] = 1;

    // set inverse skew matrix of A
    skewAInverse = Skew(A);
    nmrInverse(skewAInverse);

    // pointers to kinematics
    CurrentKinematics = Kinematics.at(0);
    DesiredKinematics = Kinematics.at(1);

//    std::cout << "Current Translation: " << CurrentFrame.Translation() << std::endl;
//    std::cout << "Desired Translation: " << DesiredKinematics->Frame.Translation() << std::endl;

    vctDoubleMat CurrentFrameInverse(3,3);
    CurrentFrameInverse.Assign(CurrentFrame.Rotation());
    nmrInverse(CurrentFrameInverse);

    // current kinematics gives us current frame
    CurrentFrame = CurrentKinematics->Frame;

    // desired kinematics gives us desired frame
    DesiredFrame = DesiredKinematics->Frame;

    // Put Jacobian into matrix ref
    ObjectiveMatrixRef.Assign(CurrentKinematics->Jacobian);

    // p_des - p_cur
    TranslationObjectiveVector.SetSize(3);
    TranslationObjectiveVector.Assign(DesiredFrame.Translation() - CurrentFrame.Translation());

    TranslationObjectiveVector.SetAll(0.0);
//    TranslationObjectiveVector.at(0) = 0.001;

    vctDoubleVec Alpha;
    Alpha.SetSize(3);

    RotationObjectiveVector.SetSize(3);

    vctDoubleMat rMatrix(3,3);
    rMatrix.Assign(CurrentFrameInverse * vctDoubleMat(DesiredFrame.Rotation()));


//    // TESTING
//    vctDoubleVec rotCurrent = GetRPY(vctDoubleMat(CurrentFrame.Rotation()));
//    vctDoubleVec rotDesired = GetRPY(vctDoubleMat(DesiredFrame.Rotation()));

//    RotationObjectiveVector = rotDesired - rotCurrent;

//    /////////

//    std::cout << "Matrix \n" << rMatrix << std::endl;
//    RotationObjectiveVector = GetAxisAngle(rMatrix);
//    RotationObjectiveVector = GetEulerAngle(rMatrix);
    RotationObjectiveVector = GetRPY(rMatrix);


//    RotationObjectiveVector.SetAll(0);  // Test
//    RotationObjectiveVector.at(0) = 0.4;

    // fill v_T and v_R into vector ref
    for(size_t i = 0; i < 3; i++)
    {
        ObjectiveVectorRef[i] = TranslationObjectiveVector[i];
    }
    for(size_t i = 3; i < 6; i++)
    {
        ObjectiveVectorRef[i] = RotationObjectiveVector[i-3];
    }
    ObjectiveVectorRef[6] = CurrentKinematics->JointState->JointPosition[6];


    // make conversion, if necessary
    ConvertRefs(mode,TickTime);

//    std::cout << "Mat Obj \n" << ObjectiveMatrixRef << std::endl;
//    std::cout << "Vec Obj \n" << ObjectiveVectorRef << std::endl;

}

vctDoubleVec mtsVFFollowJacobian::GetAxisAngle(const vctDoubleMat &m)
{
    vctDoubleVec axis;
    axis.SetSize(3);

    std::cout << "cos value " << (m.at(0,0) + m.at(1,1) + m.at(2,2) - 1.0 )/2.0 << std::endl;

    double angle = acos((m.at(0,0) + m.at(1,1) + m.at(2,2) - 1.0 )/2.0);
    axis[0] = (m.at(2,1) - m.at(1,2))/ sqrt(pow((m.at(2,1) - m.at(1,2)),2) + pow((m.at(0,2) - m.at(2,0)),2) + pow((m.at(1,0) - m.at(0,1)),2));
    axis[1] = (m.at(0,2) - m.at(2,0))/ sqrt(pow((m.at(2,1) - m.at(1,2)),2) + pow((m.at(0,2) - m.at(2,0)),2) + pow((m.at(1,0) - m.at(0,1)),2));
    axis[2] = (m.at(1,0) - m.at(0,1))/ sqrt(pow((m.at(2,1) - m.at(1,2)),2) + pow((m.at(0,2) - m.at(2,0)),2) + pow((m.at(1,0) - m.at(0,1)),2));

    axis = axis.NormalizedSelf();

    std::cout << "Axis " << axis << std::endl;
    std::cout << "Angle " << angle << std::endl;

    axis.Multiply(angle);

    std::cout << "AxisAngle " << axis << std::endl;

    return axis;
}

vctDoubleVec mtsVFFollowJacobian::GetEulerAngle(const vctDoubleMat &m)
{
    vctDoubleVec angles;
    angles.SetSize(3);

    angles[0] = acos(-m[2][1]/(sqrt(1 - pow(m[2][2],2))));
    angles[1] = acos(m[2][2]);
    angles[2] = acos(m[1][2]/(sqrt(1 - pow(m[2][2],2))));

    std::cout << "Angles " << angles << std::endl;
    return angles;
}


vctDoubleVec mtsVFFollowJacobian::GetRPY(const vctDoubleMat &m)
{
//    vctDoubleVec angles;
//    angles.SetSize(3);

////    angles[0] = atan(m[1][0]/m[0][0]);
////    angles[1] = atan(-m[2][0]/sqrt(pow(m[2][1],2) + pow(m[2][2],2)));
////    angles[2] = atan(m[2][1]/ m[2][2]);
//    double roll, pitch, yaw;


//    roll = atan2(m[2][1], m[2][2]);
//    yaw = atan2(m[1][0], m[0][0]);

//    pitch = atan2(-m[2][0], cos(yaw)*m[0][0] + sin(yaw) * m[1][0] );

//    angles[0] = roll;
//    angles[1] = pitch;
//    angles[2] = yaw;

//    std::cout << "Angles " << angles << std::endl;

//    return angles;

    vctDoubleVec angles;
    angles.SetSize(3);
    double beta, alpha, gamma;

    beta = atan2(-m[2][0], sqrt( pow(m[0][0], 2) + pow(m[1][0], 2)));

    if (beta == cmnPI_2)
    {
        alpha = 0;
        gamma = atan2(m[0][1], m[1][1]);
    }
    else if (beta == -cmnPI_2)
    {
        alpha = 0;
        gamma = -atan2(m[0][1], m[1][1]);
    }
    else
    {
        alpha = atan2(m[1][0], m[0][0]);
        gamma = atan2(m[2][1], m[2][2]);
    }

//    roll = gamma;
//    pitch = beta;
//    yaw = alpha;

    angles[0] = gamma;
    angles[1] = beta;
    angles[2] = alpha;

    return angles;
}
