/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 $Id: $
 
 Author(s):  Paul Wilkening
 Created on:
 
 (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.
 
 --- begin cisst license - do not edit ---
 
 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.
 
 --- end cisst license ---
 */

#ifndef _prmSimpleRobotKinematicsState_h
#define _prmSimpleRobotKinematicsState_h

#include <sawConstraintController/prmKinematicsState.h>

//! This is a collection of joint and cartesian data relating to a frame
/*! \brief prmSimpleRobotKinematicsState: A class that makes updating nmrVFData with the robot state at a given point easier
 */
class prmSimpleRobotKinematicsState : public prmKinematicsState
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public: 

	/*! Constructor	
	*/
    prmSimpleRobotKinematicsState() : prmKinematicsState(){Update();}
	/*! Constructor	
	*/
    prmSimpleRobotKinematicsState(std::string n, prmJointState * js) : prmKinematicsState(n,js){Update();}    

	//! Updates the kinematics information using the pointer to a joint state. 
	/*! Update
	*/
	void Update();

};

CMN_DECLARE_SERVICES_INSTANTIATION(prmSimpleRobotKinematicsState)

#endif
