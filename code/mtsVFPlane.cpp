/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Preetham Chalasani
  Created on: 2014

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawConstraintController/mtsVFPlane.h>

CMN_IMPLEMENT_SERVICES(mtsVFPlane)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFPlane::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{    
    /*
         Fill in refs
         Plane equation : ax + by + cz = d
         [a,b,c].[x,y,z] = d, where [a,b,c] is the normal of the plane and [x,y,z] is a point on the plane
         d = vctDotProduct(Normal, PointOnPlane)

         min || N.(x+dx) - d|| => min || N.dx - (d - N.x)||
         where,
            N is the normal vector represented as 1x3 matrix
            x is the current position
            d is the constant calculated using above formula
    */

    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Plane VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Plane VF given improper input");
    }

    // Pointer to kinematics
    CurrentKinematics = Kinematics.at(0);
    vct3 CurrentPos(CurrentKinematics->Frame.Translation());

    mtsVFDataPlane *planeData = (mtsVFDataPlane*)(Data);

    if(!planeData)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Plane data object not set" << std::endl;
        return;
    }

    vctDynamicMatrix<double> N( 1, 3, VCT_COL_MAJOR );
    N.SetAll(0);
    vct1 d;
    if(IsFrameSet)
    {
        vctFixedSizeVector<double,3> xyz = frame.Translation();
        N[0][0] = frame[0][2];
        N[0][1] = frame[1][2];
        N[0][2] = frame[2][2];
        d = vct1( N[0][0]*xyz[0] + N[0][1]*xyz[1] + N[0][2]*xyz[2] );
        IsFrameSet = false;
    }

    else
    {
        N[0][0] = planeData->Normal[0];
        N[0][1] = planeData->Normal[1];
        N[0][2] = planeData->Normal[2];
        d = vct1(vctDotProduct(planeData->Normal, planeData->PointOnPlane));
    }    


    vctDynamicMatrix<double> Jacobian3x6( 3, 6, VCT_COL_MAJOR );
    for (int i = 0; i < Jacobian3x6.rows(); ++i)
        for (int j = 0; j < Jacobian3x6.cols(); ++j)
            Jacobian3x6.at(i,j) = CurrentKinematics->Jacobian.at(i,j);

    IneqConstraintVectorRef.Assign(vct1(d - vct1(vctDotProduct(planeData->Normal, CurrentPos))));

    IneqConstraintMatrixRef.Assign(N * Jacobian3x6);
//    IneqConstraintMatrixRef.Assign(N);


//    std::cout << "Mat Ine \n" << IneqConstraintMatrixRef << std::endl;
//    std::cout << "Vec Ine \n" << IneqConstraintVectorRef << std::endl;
//    @TODO Fix convert Refs
//    ConvertRefs(mode,TickTime);
}

void mtsVFPlane::SetFrame(const vctFrame4x4<double> &Frame)
{
    IsFrameSet = true;
    frame = Frame;
}
