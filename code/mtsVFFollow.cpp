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
    CurrentKinematics = Kinematics.at(0);
    DesiredKinematics = Kinematics.at(1);

    // current kinematics gives us current joint set
    CurrentJointSet.SetSize(7);
    CurrentJointSet.Assign(CurrentKinematics->JointState->JointPosition);

    // desired kinematics gives us desired frame
    DesiredFrame.FromNormalized(DesiredKinematics->Frame);

    // use desired frame to solve for desired joint set
    DesiredJointSet.SetSize(6);
    DesiredJointSet.Assign(CurrentJointSet,6);

    if(Manipulator)
    {
        if(Manipulator->InverseKinematics(DesiredJointSet, DesiredFrame) == robManipulator::ESUCCESS)
        {
            DesiredJointSet.resize(7);
            DesiredJointSet[6] = CurrentJointSet[6];

            ObjectiveMatrixRef.Diagonal().SetAll(1.0);

            ObjectiveVectorRef.Assign(DesiredJointSet - CurrentJointSet);

            ConvertRefs(mode,TickTime);
        }
    }
    else
    {
        cmnThrow("FillInTableauRefs: Inverse Kinematics failed");
    }

}
