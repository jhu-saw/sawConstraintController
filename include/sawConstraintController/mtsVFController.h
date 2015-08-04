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

#ifndef _mtsVFController_h
#define _mtsVFController_h

#include <cisstNumerical/nmrConstraintOptimizer.h>

#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFJointVel.h>
#include <sawConstraintController/mtsVFJointPos.h>
#include <sawConstraintController/mtsVFDataSensorCompliance.h>
#include <sawConstraintController/mtsVFSensorCompliance.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <sawConstraintController/prmSensorState.h>
#include <sawConstraintController/prmOffsetState.h>
#include <sawConstraintController/mtsVFCartVel.h>
#include <sawConstraintController/mtsVFCartOrientationVel.h>
#include <sawConstraintController/mtsVFPlane.h>
#include <sawConstraintController/mtsVFFollow.h>
#include <typeinfo>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

/*! \brief mtsVFController: A class that is responsible for managing the virtual fixtures, relevant state data, and the control optimizer
 */
class CISST_EXPORT mtsVFController: public cmnGenericObject
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

protected:

    //map between string names and pointers to virtual fixtures
    std::map<std::string, mtsVFBase *> VFMap;

    //map between string names and pointers to kinematics objects
    std::map<std::string, prmKinematicsState *> Kinematics;

    //map between string names and pointers to sensor objects
    std::map<std::string, prmSensorState *> Sensors;

    //robot joint state
    prmJointState JointState;

    //control optimizer variables
    nmrConstraintOptimizer Optimizer;

    //Current mode of controller (what controllerOutput represents)
    //The possible values of MODE refer to:
    //1. Treating controllerOutput as an incremental joint position
    //2. Treating controllerOutput as an incremental cartesian position
    mtsVFBase::CONTROLLERMODE ControllerMode;

public:

    /*! Constructor
    */
    mtsVFController(){}

    /*! Constructor
    */
    mtsVFController(size_t num_joints, mtsVFBase::CONTROLLERMODE cm):
        Optimizer(num_joints)
    {
        ControllerMode = cm;
    }

    nmrConstraintOptimizer GetOptimizer(){return Optimizer;}

protected:

    //! Adds/Updates a vf data object
    void AddVFJointVelocity(const mtsVFDataBase & vf);

    //! Adds/Updates a vf data object
    void AddVFJointPosition(const mtsVFDataBase & vf);

    //! Adds/Updates a vf data object
    void AddVFCartesianTranslation(const mtsVFDataBase & vf);

    //! Adds/Updates a vf data object
    void AddVFCartesianOrientation(const mtsVFDataBase & vf);

    //! Adds/Updates a vf data object
    void AddVFSensorCompliance(const mtsVFDataSensorCompliance & vf);

    //! Adds/Updates a vf plane object
    void AddVFPlane(const mtsVFDataPlane &vf);        

    void AddVFFollowPath(const mtsVFDataBase & vf);

    //! Adds/Updates a sensor to the map
    void SetSensor(const prmSensorState & sen);

    //! Adds/Updates a sensor to the map
    void SetSensorOffset(const prmOffsetState & sen);

    //! Removes a sensor from the map
    void RemoveSensorFromMap(const std::string & senName);

    //! Changes the variable the optimizer is solving for
    void SetMode(const mtsVFBase::CONTROLLERMODE & m);

    //! Finds the "base" object for kinematics and sensor data that has an offset
    void LookupBaseData(void);

    //! Adds/Updates a kinematics object to the map
    void SetKinematics(const prmKinematicsState & kin);

    //! Removes a kinematics object from the map
    void RemoveKinematicsFromMap(const std::string & kinName);

    //! Updates the robot state data and control optimizer
    void UpdateOptimizer(double TickTime);

    //! Solves the constraint optimization problem and fills the result into the parameter
    nmrConstraintOptimizer::STATUS Solve(vctDoubleVec & dq);

    //! Helper function that increments users of new vf
    void IncrementUsers(const std::vector<std::string> kin_names, const std::vector<std::string> sensor_names);

    //! Helper function that decrements users of new data in an old vf
    void DecrementUsers(const std::vector<std::string> kin_names, const std::vector<std::string> sensor_names);

    bool SetVFData(const mtsVFDataBase & data, const std::type_info & type);

    bool SetVFDataSensorCompliance(const mtsVFDataSensorCompliance & data, const std::type_info & type);

    bool SetVFDataPlane(const mtsVFDataPlane & data, const std::type_info & type);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFController);

#endif // _mtsVFController_h
