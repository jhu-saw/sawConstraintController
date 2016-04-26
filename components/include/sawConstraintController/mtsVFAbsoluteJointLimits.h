/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 Author(s):  Paul Wilkening
 Created on: 2015

 (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsVFAbsoluteJointLimits_h
#define _mtsVFAbsoluteJointLimits_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFJointPos.h>
#include <sawConstraintController/mtsVFDataAbsoluteJointLimits.h>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

/*! \brief mtsVFAbsoluteJointLimits: A class that contains logic for the implementation of joint limits
 */
class CISST_EXPORT mtsVFAbsoluteJointLimits : public mtsVFJointPosition
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

public:

    /*! Constructor
    */
    mtsVFAbsoluteJointLimits() : mtsVFJointPosition(){}

    /*! Constructor
    \param name String name of object
    */
    mtsVFAbsoluteJointLimits(const std::string & name, mtsVFDataAbsoluteJointLimits * data) : mtsVFJointPosition(name,data){}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFAbsoluteJointLimits);

#endif // _mtsVFAbsoluteJointLimits_h
