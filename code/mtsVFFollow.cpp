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
//std::cout << "Start FITR" << std::endl;
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

    //dx_translation = vct3(0.421444,0.252093,0.0673298) - CurrentFrame.Translation();
    //dx_translation = vct3(0.423517,0.268144,0.0487406) - CurrentFrame.Translation();
    //dx_translation = vct3(0.421394,0.270174,0.0476985) - CurrentFrame.Translation();

    vctDoubleMat JacResized = CurrentKinematics->Jacobian;
    JacResized.resize(3,7);
    JacResized.Column(6).SetAll(0.0);


//std::cout << ObjectiveVectorRef.size() << " " << dx.size() << std::endl;
//std::cout << ObjectiveMatrixRef.sizes() << " " << JacResized.sizes() << std::endl;
    // Delta x
    ObjectiveVectorRef.Assign(dx_translation);
    // Put Jacobian into matrix ref
    ObjectiveMatrixRef.Assign(JacResized);

    // make conversion, if necessary
    ConvertRefs(mode,TickTime);
//std::cout << "End" << std::endl;
}
