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

#ifndef _mtsVFSensorCompliance_h
#define _mtsVFSensorCompliance_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <sawConstraintController/prmSensorState.h>
#include <sawConstraintController/mtsVFDataSensorCompliance.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFJointVel.h>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

//! This is the base class for all virtual fixture objects
/*! \brief mtsVFSensorCompliance: A class that contains logic for the implementation of virtual fixtures
 */
class CISST_EXPORT mtsVFSensorCompliance: public mtsVFJointVelocity
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

public:

    /*! Constructor
    */
    mtsVFSensorCompliance() : mtsVFJointVelocity(){}

    mtsVFSensorCompliance(mtsVFDataSensorCompliance *data) : mtsVFJointVelocity(DefaultKinematicsName,data)
    {
        Data->ObjectiveRows = data->ObjectiveRows;
    }

    /*! Constructor
    \param name String name of object
    */
    mtsVFSensorCompliance(const std::string & name, mtsVFDataSensorCompliance * data) : mtsVFJointVelocity(name,data)
    {
        //assuming gain is properly sized for now, if not will throw an error later
        Data->ObjectiveRows = data->ObjectiveRows;
    }

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double tickTime);

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double tickTime);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFSensorCompliance);

#endif // _mtsVFSensorCompliance_h
