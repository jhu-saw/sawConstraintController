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
    
    @brief Calculate joint angles that take the robot from the current position and orientation to a desired one.

    This is useful for having the arm follow a path,
    particularly one that leads the end effector slightly 
    at each instant to indicate the new destination.

    fill in refs
    min || J*dq - [v_T ; v_R] ||
    v_T = p_des - R_des * R^(-1)_cur * p_cur
    v_R = sk(A)^(-1) * A - sk(A)^(-1) * R_des * R^(-1)_cur * A
    J is the jacobian, p_des is the desired frame's translation,
    R_des is the desired frame's rotation, R^(-1)_cur is the inverse of the current frame's rotation,
    p_cur is the current frame's translation, A is an arbitrary vector, sk(A)^(-1) is the inverse of the skew matrix of A
    

   The goal is constructing the dx parameter which contains:
   [x, y, z, rx*theta, ry*theta, rz*theta]
   This is the same xdot format output by J*qdot
   that way we try to make xdot - J*qdot == [0,0,0,0,0,0].transpose()
   which means that you've found the joint angles that
   will put you at exactly the desired position and rotation
*/
void mtsVFFollow::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{


    //check desired frame, current frame dependencies
    if(Kinematics.size() < 2)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Follow VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Follow VF given improper input");
    }

    // pointers to kinematics
    CurrentKinematics = Kinematics.at(0);
    DesiredKinematics = Kinematics.at(1);

    // Current Frame
    vctFrm3 CurrentFrame;
    CurrentFrame.FromNormalized(CurrentKinematics->Frame);

    // Desired Frame
    vctFrm3 DesiredFrame;
    DesiredFrame.FromNormalized(DesiredKinematics->Frame);


    vct3 dx_translation, dx_rotation;


    // Translation Part
    dx_translation = DesiredFrame.Translation() - CurrentFrame.Translation();

    // Resized jacobian
    vctDoubleMat JacResized(CurrentKinematics->Jacobian.rows(),CurrentKinematics->Jacobian.cols());
    JacResized.Assign(CurrentKinematics->Jacobian);
    JacResized.resize(3,CurrentKinematics->Jacobian.cols());

    // Rotation part
    vctAxAnRot3 dxRot; // axis angle format
    // (Cur^-1*Desired) => change in rotation => convert to AxisAngle
    dxRot.FromNormalized(((CurrentFrame.Inverse() * DesiredFrame).Rotation()));
    // Put Axis Angle object into a normal vector
    dx_rotation = dxRot.Axis() * dxRot.Angle();
    // Flip and rotate the vector around which rotation occurs
    // Note: depending on what you define as positive theta,
    //       you may need to account for the -CurrentFrame.Rotation()
    //       so that it is the equivalent of +CurrentFrame.Rotation().
    dx_rotation = CurrentFrame.Rotation() * dx_rotation;

    if(Data->ObjectiveRows == 3)
    {
        // Delta x
        ObjectiveVectorRef.Assign(dx_translation);
        // Put Jacobian into matrix ref
        ObjectiveMatrixRef.Assign(JacResized);
    }
    else if(Data->ObjectiveRows == 6)
    {
        // Constructing the dx parameter which contains:
        // [x, y, z, rx*theta, ry*theta, rz*theta]
        // This is the same xdot format output by J*qdot
        // that way we try to make xdot - J*qdot == [0,0,0,0,0,0].transpose()
        // which means that you've found the joint angles that
        // will put you at exactly the desired position and rotation
        vctDoubleVec dx(6);
        std::copy(dx_translation.begin(), dx_translation.end(), dx.begin() );
        std::copy(dx_rotation.begin() , dx_rotation.end() , dx.begin()+3);
        ObjectiveVectorRef.Assign(dx);
        ObjectiveMatrixRef.Assign(CurrentKinematics->Jacobian);
    }
    else
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Follow VF given improper number of rows" << std::endl;
        cmnThrow("FillInTableauRefs: Follow VF given improper number of rows");
    }

    // make conversion, if necessary
    ConvertRefs(mode,TickTime);

}
