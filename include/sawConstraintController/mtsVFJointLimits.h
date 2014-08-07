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

#ifndef _mtsVFJointLimits_h
#define _mtsVFJointLimits_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFJointPos.h>
#include <sawConstraintController/mtsVFDataJointLimits.h>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

/*! \brief mtsVFJointLimits: A class that contains logic for the implementation of  Plane virtual fixtures
 */
class CISST_EXPORT mtsVFJointLimits : public mtsVFJointPosition
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

public:

    /*! Constructor
    */
    mtsVFJointLimits() : mtsVFJointPosition(){}

    /*! Constructor
    \param name String name of object
    */
    mtsVFJointLimits(const std::string & name, mtsVFDataJointLimits * data) : mtsVFJointPosition(name,data){}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFJointLimits);

#endif // _mtsVFJointLimits_h
