#ifndef _prmDaVinciKinematicsState_h
#define _prmDaVinciKinematicsState_h


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

#include <sawConstraintController/prmKinematicsState.h>
#include <cisstRobot/robManipulator.h>

//! This is a collection of joint and cartesian data relating to a frame
/*! \brief prmDaVinciKinematicsState: A class that makes updating nmrVFData with the robot state at a given point easier
 */
class prmDaVinciKinematicsState : public prmKinematicsState
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public: 

    vctDoubleVec Joints;

    vctFrm3 Frame6to7Inverse;

    double DesiredOpenAngle;

    robManipulator Manipulator;

	/*! Constructor	
	*/
    prmDaVinciKinematicsState() : prmKinematicsState()
    {
        vctFrm4x4 Frame6to7;
        Frame6to7.Assign(0.0, -1.0,  0.0, 0.0,
                         0.0,  0.0,  1.0, 0.0102,
                         -1.0, 0.0,  0.0, 0.0,
                         0.0,  0.0,  0.0, 1.0);
        Frame6to7 = Frame6to7.Inverse();
        Frame6to7Inverse.From(Frame6to7);
    }
	/*! Constructor	
	*/
    prmDaVinciKinematicsState(std::string n, vctFrm3 * frm);
	//! Updates the kinematics information using the pointer to a joint state. 
	/*! Update
	*/
    void Update(){}

    bool InverseKinematics(vctDynamicVector<double>& q,
                           const vctFrame4x4<double>& Rts,
                           double tolerance=1e-12,
                           size_t Niteration=1000,
                           double LAMBDA=0.001);

};

CMN_DECLARE_SERVICES_INSTANTIATION(prmDaVinciKinematicsState)

#endif
