/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 $Id: $

 Author(s):  Paul Wilkening
 Created on:

 (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawConstraintController/mtsVFSideview.h>

CMN_IMPLEMENT_SERVICES(mtsVFSideview)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFSideview::FillInTableauRefs(const CONTROLLERMODE, const double)
{    
#if 0
    // fill in refs
    // min || [I | -skew(p-w)] * J - (w-v) ||
    // p = Probe Tip
    // v = VFPoint
    // w = Projected Point

    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error: Sideview VF given improper input" << std::endl;
        cmnThrow("Error: Sideview VF given improper input");
    }

    mtsVFDataSideview * SideviewData = (mtsVFDataSideview *)(Data);
    vctFrm3 * FrmP = &(Kinematics.at(0)->Frame);
    vctDoubleMat * JacP = &(Kinematics.at(0)->Jacobian);
    vct3 * VFPoint = &(SideviewData->VFPoint);

    //Rz is the probe's axis
    vct3 Rz = FrmP->Rotation()*vct3(0,0,1);

    //probe tip to current point vector
    vct3 pv;
    pv = VFPoint - FrmP->Translation();

    //probe tip to current point's projection onto Rz
    vct3 pw = Rz*vctDotProduct(Rz,pv);

    //vector we want to minimize (objective's vector)
    vct3 wv = pv-pw;

    //set up objective's matrix
    //[I | -skew(pw)]*J
    Jw(0,4) = pw[2]; Jw(0,5) = -pw[1];
    Jw(1,3) = -pw[2];  Jw(1,5) = pw[0];
    Jw(2,3) = pw[1]; Jw(2,4) = -pw[0];

    ObjectiveMatrixRef.Assign(Jw*(*JacP));

    ObjectiveVectorRef.Assign(wv);

    ConvertVariables(mode,TickTime);

    for(size_t i = 0; i < Data->NumSlacks; i++)
    {
        ObjectiveMatrixSlackRef.Column(i).SetAll(Data->SlackCosts[i]);
        IneqConstraintMatrixRef[i][i] = 1;
    }
#endif
}
