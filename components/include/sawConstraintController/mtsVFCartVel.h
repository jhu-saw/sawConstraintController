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

#ifndef _mtsVFCartVel_h
#define _mtsVFCartVel_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFBase.h>

//! This is the base class for all virtual fixture objects
/*! \brief mtsVFCartVel: A class that contains logic for the implementation of virtual fixtures
 */
class mtsVFCartesianTranslation: public mtsVFBase
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public:

    vctDoubleMat IdentitySkewMatrix;
    vctDoubleVec w;

    /*! Constructor
    */
    mtsVFCartesianTranslation():
	mtsVFBase()
    {}

    /*! Constructor
    \param name String name of object
    \param data VF Data passed from remote user
    */
    mtsVFCartesianTranslation(const std::string & name, mtsVFDataBase * data):
	mtsVFBase(name, data)
    {}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void AssignRefs(const mtsVFBase::CONTROLLERMODE, const double, const vctDoubleVec & ,vctDoubleMat &, vctDoubleVec &, vctDoubleMat &, vctDoubleVec &, vctDoubleMat &, vctDoubleVec &);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFCartesianTranslation)

#endif // _mtsVFCartVel_h
