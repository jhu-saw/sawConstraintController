/*
  Author(s): Henry Phalen, based on previous work by Paul Wilkening and others.
  Created on: 2022-10-05

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef MTSCONSTRAINTABSOLUTEJOINTLIMITS_H
#define MTSCONSTRAINTABSOLUTEJOINTLIMITS_H

#include <sawConstraintController/mtsConstraintBase.h>
#include <sawConstraintController/prmKinematicsState.h>

class mtsConstraintAbsoluteJointLimits : public mtsConstraintBase
{
public:
    mtsConstraintAbsoluteJointLimits(const Json::Value &constraint_config, const kinematics_map kins_map);
    ~mtsConstraintAbsoluteJointLimits(){};
    void FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime);
    void PrepareDataForBridging(mtsInterfaceProvided* provided);

    void set_active(const bool &active) // needs to be const ref for provided interface
    {
        Active = active;
        std::cout << Name << " set_active: " << active << std::endl;
    }
    void set_lower_limits(const vctDoubleVec &limits)
    {
        if (check_limit_input(limits))
        {
            lower_limits = limits;
            std::cout << Name << " set_lower_limits: " << limits << std::endl;
        }
    }
    void set_upper_limits(const vctDoubleVec &limits)
    {
        if (check_limit_input(limits))
        {
            upper_limits = limits;
            std::cout << Name << " set_upper_limits: " << limits << std::endl;
        }
    }

    prmKinematicsState *kinematics;
    vctDoubleVec lower_limits;
    vctDoubleVec upper_limits;
    size_t num_joints;

private:
    bool check_limit_input(const vctDoubleVec &limits)
    {
        if (limits.size() != num_joints)
        {
            std::cerr << "[mtsConstraintAbsoluteJointLimits]: Given input size: " << limits.size() << " does not match number of joints: " << num_joints << std::endl;
            return 0;
        }
        return 1;
    }
};

#endif // MTSCONSTRAINTABSOLUTEJOINTLIMITS_H
