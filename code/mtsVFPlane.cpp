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

#include <sawConstraintController/mtsVFPlane.h>

CMN_IMPLEMENT_SERVICES(mtsVFPlane)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFPlane::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    // fill in refs
    // min || I*dq - (q_des - q_curr) ||
    // I is the identity matrix, q_des is the desired joint set, q_curr is the current joint set

    //check desired frame, current frame dependencies

    mtsVFDataPlane *planeData = (mtsVFDataPlane*)(Data);

    vct1 d(vctDotProduct(planeData->Normal, planeData->PointOnPlane));

    vctDynamicMatrix<double> N( 1, 3, VCT_COL_MAJOR );
    N[0][0] = planeData->Normal[0];
    N[0][1] = planeData->Normal[1];
    N[0][2] = planeData->Normal[2];

    ObjectiveVectorRef.Assign(d);
    ObjectiveMatrixRef.Assign(N);

    ConvertRefs(mode,TickTime);


}
