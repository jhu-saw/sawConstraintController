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

#ifndef _mtsVFSideview_h
#define _mtsVFSideview_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <sawConstraintController/prmSensorState.h>
#include <sawConstraintController/mtsVFDataSensorCompliance.h>
#include <sawConstraintController/mtsVFJointVel.h>
#include <sawConstraintController/mtsVFDataSideview.h>

//! This is the base class for all virtual fixture objects
/*! \brief mtsVFSideview: A class that contains logic for the implementation of virtual fixtures
 */
class mtsVFSideview : public mtsVFJointVelocity
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

private:

    // pointer to probe tip frame
    vctFrm3 * FrmP;
    // pointer to jacobian of probe tip frame
    vctDoubleMat * JacP;
    // pointer to the cochlear center point given by the sideview VF data
    vct3 * VFPoint;
    // will store [I | -sk(pw)]
    vctDoubleMat IdentitySkew;
    // axis of probe
    vct3 Rz;
    // projection of VFPoint onto probe axis
    vct3 ProjectedPoint;
    // pointer to probe tip location
    vct3 * ProbeTip;

public:

    /*! Constructor
    */
    mtsVFSideview() : mtsVFJointVelocity(){}

    mtsVFSideview(mtsVFDataSensorCompliance *data) : mtsVFJointVelocity(DefaultKinematicsName,data){}

    /*! Constructor
    \param name String name of object
    */
    mtsVFSideview(const std::string & name, mtsVFDataSideview * data) : mtsVFJointVelocity(name,data){}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFSideview)

#endif
