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

CMN_IMPLEMENT_SERVICES(mtsVFFollow)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFFollow::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    std::cout << "Being Called " << std::endl;
    // fill in refs    
    // min || I*dq - (q_des - q_curr) ||
    // I is the identity matrix, q_des is the desired joint set, q_curr is the current joint set

    //check desired frame, current frame dependencies
    if(Kinematics.size() < 2)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Follow VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Follow VF given improper input");
    }

    // pointers to kinematics
    CurrentKinematics = (prmDaVinciKinematicsState *)(Kinematics.at(0));
    DesiredKinematics = (prmDaVinciKinematicsState *)(Kinematics.at(1));

    // current kinematics gives us current joint set
    CurrentJointSet = CurrentKinematics->Joints;

    std::cout << "C Joint " << CurrentJointSet << std::endl;

    // desired kinematics gives us desired frame
    DesiredFrame = DesiredKinematics->Frame * DesiredKinematics->Frame6to7Inverse;
    DesiredFrame4x4.FromNormalized(DesiredFrame);

    // use desired frame to solve for desired joint set
    DesiredJointSet.SetSize(6);
    DesiredJointSet.SetAll(0.0);

    if(DesiredKinematics->Manipulator.InverseKinematics(DesiredJointSet, DesiredFrame4x4))
    {
        DesiredJointSet.resize(7);
        DesiredJointSet[6] = DesiredKinematics->DesiredOpenAngle;


        std::cout << "D Joint " << DesiredJointSet << std::endl;

        ObjectiveMatrixRef.Diagonal().SetAll(1.0);

        std::cout << "Matrix  " << ObjectiveMatrixRef << std::endl;

        ObjectiveVectorRef.Assign(DesiredJointSet - CurrentJointSet);

        std::cout << "Vector  " << ObjectiveVectorRef << std::endl;

        ConvertRefs(mode,TickTime);
    }
    else
    {
        cmnThrow("FillInTableauRefs: Inverse Kinematics failed");
    }

}
