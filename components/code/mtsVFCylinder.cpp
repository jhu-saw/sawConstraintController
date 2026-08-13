/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Max Zhaoshuo Li

  (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawConstraintController/mtsVFCylinder.h>

mtsVFCylinder::mtsVFCylinder(const std::string & name, mtsVFDataBase * data):
    mtsVFBase(name, data)
{

}

void mtsVFCylinder::FillInTableauRefs(const mtsVFBase::CONTROLLERMODE CMN_UNUSED(mode), const double CMN_UNUSED(tickTime))
{
    // ||v|| + dx . v >= R
    mtsVFDataCylinder * cylinderData = reinterpret_cast<mtsVFDataCylinder*>(Data);
    if (!cylinderData){
        std::cout << "Data type does not match!" << std::endl;
        cmnThrow("Data type does not match!");
        return;
    }

    // TODO: check how many kin are there

    CurrentKinematics = Kinematics.at(0);
    vctDoubleVec CurrentPos(vctDoubleVec(CurrentKinematics->Frame.Translation()));

    // compute current directional vector
    vctDoubleVec closestPoint = ClosestLinePoint(CurrentPos, vctDoubleVec(cylinderData->Point), vctDoubleVec(cylinderData->Axis));
    vct3 lineToCurrPos = vct3(CurrentPos - closestPoint);
    vctDynamicMatrix<double> N( 1, 3, VCT_COL_MAJOR );
    N[0][0] = lineToCurrPos[0];
    N[0][1] = lineToCurrPos[1];
    N[0][2] = lineToCurrPos[2];

    // compute matrix
    IneqConstraintMatrixRef.ProductOf(N,CurrentKinematics->Jacobian.Ref(3,cylinderData->NumJoints,0,0));

    // compute vector
    IneqConstraintVectorRef.Assign(vct1(cylinderData->Radius-lineToCurrPos.Norm()));
}

void mtsVFCylinder::ConvertRefs(const mtsVFBase::CONTROLLERMODE CMN_UNUSED(mode), const double CMN_UNUSED(tickTime))
{

}
