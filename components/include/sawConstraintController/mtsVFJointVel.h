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

#ifndef _mtsVFJointVel_h
#define _mtsVFJointVel_h

#include <sawConstraintController/mtsVFBase.h>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

//! This is the base class for all virtual fixture objects
/*! \brief mtsVFJointVel: A class that contains logic for the implementation of virtual fixtures
 */
class CISST_EXPORT mtsVFJointVelocity: public mtsVFBase
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

public:

    /*! Constructor
    */
    mtsVFJointVelocity():
	mtsVFBase()
    {}

    /*! Constructor
    \param name String name of object
    \param data Data passed from remote user
    */
    mtsVFJointVelocity(const std::string & name, mtsVFDataBase * data):
	mtsVFBase(name, data)
    {}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFJointVelocity);

#endif // _mtsVFJointVel_h
