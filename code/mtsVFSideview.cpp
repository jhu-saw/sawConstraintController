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

#include <sawConstraintController/mtsVFSideview.h>

CMN_IMPLEMENT_SERVICES(mtsVFSideview);

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFSideview::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    // fill in refs
    // We want to minimize the distance between a point in the center of the cochlea and its projection
    // onto the sideview axis
    // We aim to minimize this expression with our choice of dq
    // min || [I | -sk(p)] * Jac * dq + w + p - v)
    // p = Sideview Probe Tip
    // v = VFPoint (point at center of cochlea saved in VF data object)
    // w = Projected Point (VFPoint projected onto probe axis)
    // Jac = the jacobian at the probe tip
    // dq = the incremental joint movement of the robot

    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error: Sideview VF given improper input" << std::endl;
        cmnThrow("Error: Sideview VF given improper input");
    }

    // sideview specific VF data object (need this conversion to access VFPoint)
    mtsVFDataSideview * SideviewData = (mtsVFDataSideview *)(Data);
    // pointer to probe tip frame
    FrmP = &(Kinematics.at(0)->Frame);
    // pointer to jacobian
    JacP = &(Kinematics.at(0)->Jacobian);
    // pointer to cochlear center point for this VF object (v)
    VFPoint = &(SideviewData->VFPoint);
    // probe tip (p)
    ProbeTip = &(FrmP->Translation());
    // axis of probe
    Rz = FrmP->Rotation()*vct3(0,0,1);
    // projected point (w) by projecting VFPoint-ProbeTip onto Rz to get ProjectedPoint - ProbeTip
    // add ProbeTip to get ProjectedPoint
    ProjectedPoint = Rz*vctDotProduct(Rz,*VFPoint-*ProbeTip) + *ProbeTip;

    //set up objective's matrix
    //[I | -skew(p)]
    IdentitySkew.SetSize(3,6);
    IdentitySkew.SetAll(0.0);
    IdentitySkew(0,0) = 1;
    IdentitySkew(1,1) = 1;
    IdentitySkew(2,2) = 1;
    IdentitySkew(0,4) = (*ProbeTip)(2); IdentitySkew(0,5) = -(*ProbeTip)(1);
    IdentitySkew(1,3) = -(*ProbeTip)(2);  IdentitySkew(1,5) = (*ProbeTip)(0);
    IdentitySkew(2,3) = (*ProbeTip)(1); IdentitySkew(2,4) = -(*ProbeTip)(0);

    // [I | -sk(p)] * Jac
    ObjectiveMatrixRef.Assign(IdentitySkew*(*JacP));

    // w + p - v
    ObjectiveVectorRef.Assign(ProjectedPoint + *ProbeTip - *VFPoint);

    ConvertRefs(mode,TickTime);

    for(size_t i = 0; i < Data->NumSlacks; i++)
    {
        ObjectiveMatrixSlackRef.Column(i).SetAll(Data->SlackCosts(i));
        IneqConstraintMatrixRef(i,i) = 1;
    }

}
