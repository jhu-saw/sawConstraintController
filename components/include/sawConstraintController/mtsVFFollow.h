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
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFJointPos.h>

/*! @brief Calculate joint angles that take the robot from the current position and orientation to a desired one.

    This is useful for having the arm follow a path,
    particularly one that leads the end effector slightly 
    at each instant to indicate the new destination.

    fill in refs
    min || J*dq - [v_T ; v_R] ||
    v_T = p_des - R_des * R^(-1)_cur * p_cur
    v_R = sk(A)^(-1) * A - sk(A)^(-1) * R_des * R^(-1)_cur * A
    J is the jacobian, p_des is the desired frame's translation,
    R_des is the desired frame's rotation, R^(-1)_cur is the inverse of the current frame's rotation,
    p_cur is the current frame's translation, A is an arbitrary vector, sk(A)^(-1) is the inverse of the skew matrix of A
    

   The goal is constructing the dx parameter which contains:
   [x, y, z, rx*theta, ry*theta, rz*theta]
   This is the same xdot format output by J*qdot
   that way we try to make xdot - J*qdot == [0,0,0,0,0,0].transpose()
   which means that you've found the joint angles that
   will put you at exactly the desired position and rotation
 */
class mtsVFFollow : public mtsVFJointPosition
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public:

    prmKinematicsState * CurrentKinematics;
    prmKinematicsState * DesiredKinematics;        

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

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFFollow)

#endif
