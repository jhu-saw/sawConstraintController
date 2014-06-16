/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening
  Created on:

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsVFSenderTask_h
#define _mtsVFSenderTask_h

#include <cisstCommon/cmnPath.h>
#include <cisstNumerical/nmrConstraintOptimizer.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <sawConstraintController/prmForceOffsetState.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/mtsVFDataSensorCompliance.h>

enum{NB_FT = 2};
enum{NUM_JOINTS = 2};

//! This is an example for running code that uses the control optimizer
/*! \brief mtsVFExample: A class that shows an example of setting up virtual fixtures
 */
class mtsVFSenderTask : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

 private:

    //force vf
    mtsVFDataSensorCompliance forceVF;

    //sensors
    prmSensorState forceSensorState;

    //objects used to construct vf data objects
    vct3 MotionConstraint;
    vctDynamicVector<double> limit;
    vctDynamicVector<double> cost;
    int tickNum;

 public:

    /*! Constructor
      \param taskName String name of task object
      \param period Periodicity of task
    */
    mtsVFSenderTask(const std::string & taskName, double period);

    //! Runs once, initialize variables
    /*! Startup
     */
    void Startup(void){};

    //! Runs once, set up functions
    /*! Configure
     */
    void Configure(const std::string & filename = "");

    //! Runs every period
    /*! Run
     */
    void Run(void);

    //! Runs once, when execution of program is complete
    /*! Cleanup
     */
    void Cleanup(void){};

    void UpdateRobotStateData();

 protected:

    //Adds/updates a vf data object
    mtsFunctionWrite AddVFJointVelocity;
    mtsFunctionWrite AddVFJointPosition;
    mtsFunctionWrite AddVFCartesianTranslation;
    mtsFunctionWrite AddVFCartesianOrientation;
    //Adds/updates a vf data object
    mtsFunctionWrite AddVFSensorCompliance;
    mtsFunctionWrite AddVFFollow;
    //Removes an existing vf data object
    mtsFunctionWrite RemoveVFFromMap;

    //Adds/updates a sensor
    mtsFunctionWrite SetSensor;
    mtsFunctionWrite SetSensorOffset;
    //Removes a sensor
    mtsFunctionWrite RemoveSensorFromMap;

    //Adds/updates a kinematics object
    mtsFunctionWrite SetKinematics;
    //Removes a kinematics object
    mtsFunctionWrite RemoveKinematicsFromMap;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFSenderTask);

#endif
