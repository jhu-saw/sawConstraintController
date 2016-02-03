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

#ifndef _mtsSimpleVFController_h
#define _mtsSimpleVFController_h

#include <sawConstraintController/mtsVFController.h>
#include "prmSimpleRobotKinematicsState.h"

/*! \brief mtsSimpleVFController: A class that is responsible for managing the virtual fixtures, relevant state data, and the control optimizer
 */
class CISST_EXPORT mtsSimpleVFController: public mtsVFController
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

public:

    /*! Constructor
    */
    mtsSimpleVFController(){}

    /*! Constructor
    */
    mtsSimpleVFController(size_t num_joints, mtsVFBase::CONTROLLERMODE cm):
        mtsVFController(num_joints, cm) {}

    ~mtsSimpleVFController(){}

    //! Adds/Updates a simple kinematics object to the map
    void SetSimpleRobotKinematics(const prmSimpleRobotKinematicsState & kin);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSimpleVFController);

#endif // _mtsSimpleVFController_h
