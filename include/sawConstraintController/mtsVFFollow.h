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

#ifndef _mtsVFFollow_h
#define _mtsVFFollow_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/prmSensorState.h>
#include <sawConstraintController/mtsVFDataFollow.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFJointPos.h>
#include <cisstRobot/robManipulator.h>

//! This is the base class for all virtual fixture objects
/*! \brief mtsVFFollow: A class that contains logic for the implementation of virtual fixtures
 */
class mtsVFFollow : public mtsVFJointPosition
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public:

    prmKinematicsState * CurrentKinematics;
    prmKinematicsState * DesiredKinematics;
    vctDoubleVec CurrentJointSet;
    vctFrm4x4 DesiredFrame;
    vctDoubleVec DesiredJointSet;
    robManipulator * Manipulator;

    /*! Constructor
    */
    mtsVFFollow() : mtsVFJointPosition(){}

    /*! Constructor
    \param name String name of object
    */
    mtsVFFollow(const std::string & name, mtsVFDataBase * data) : mtsVFJointPosition(name,data){}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void SetManipulator(robManipulator * rm);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFFollow)

#endif
