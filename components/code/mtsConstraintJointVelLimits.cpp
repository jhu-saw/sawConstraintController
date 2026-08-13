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


/*
Adds an inequatlity constraint (i.e. fills in 'C' and 'd' terms) to the constained optimization problem on joint velocity x
argmin_x: ||Ax+b||, s.t. Cx>d, Ex=f

Ensures bounded joint velocity:
L > x > (U-p)*dt 

| I   0 |   | |    |  L  |
|       | . |x| >  |     |
| 0  -I |   | |    | -U  |
x: joint velocity
dt: is timestep
L,U are lower and upper joint limits, respectively
*/

#include <sawConstraintController/mtsConstraintJointVelLimits.h>

mtsConstraintJointVelLimits::mtsConstraintJointVelLimits(const Json::Value &config_param, const kinematics_map kins_map)
    : mtsConstraintBase()
{
    // This is for convenience below
    std::vector<std::string> expected_keys = {"active", "name", "lower_limits", "upper_limits", "kinematics"};

    // Make sure we're actually trying to set the right constraint: Hopefully just sanity check at this point
    if (config_param["type"] != "joint_velocity_limits")
    {
        std::cerr << "[mtsConstraintJointVelLimits] - incorrect type name: " << config_param["type"] << std::endl;
    }
    // Make sure all expected keys are present
    for (auto &key : expected_keys)
    {
        if (!config_param.isMember(key))
        {
            std::cerr << "[mtsConstraintJointVelLimits] - expected parameter named: " << key << std::endl;
        }
    }

    // Set all values coming straight from config file
    Active = config_param["active"].asBool();
    Name = config_param["name"].asString();

    cmnDataJSON<vctDoubleVec>::DeSerializeText(upper_limits, config_param["upper_limits"]);
    cmnDataJSON<vctDoubleVec>::DeSerializeText(lower_limits, config_param["lower_limits"]);

    // Set kinematics based on string matching as defined in config file
    std::vector<std::string> expected_kins_names;
    expected_kins_names.push_back(config_param["kinematics"].asString());

    // Make sure all expected kinematics are present in map
    for (auto &name : expected_kins_names)
    {
        if (kins_map.find(name) == kins_map.end())
        {
            std::cerr << "[mtsConstraintJointVelLimits] - cannot find kinematics in map: " << name << std::endl;
        }
    }

    // Set kinematics
    kinematics = kins_map.at(config_param["kinematics"].asString());

    // Inhernet to constraint
    num_joints = lower_limits.size();
    IneqConstraintRows = 2 * num_joints;
}


void mtsConstraintJointVelLimits::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode),
                                                    const double CMN_UNUSED(TickTime))
{
    IneqConstraintMatrixRef.SetAll(0.0);
    for (size_t i = 0; i < num_joints; i++)
    {
        IneqConstraintMatrixRef.at(i, i) = 1.0;
        IneqConstraintVectorRef.at(i) = lower_limits.at(i);

        IneqConstraintMatrixRef.at(i+num_joints, i) = -1.0;
        IneqConstraintVectorRef.at(i+num_joints) = -upper_limits.at(i);
    }
}

void mtsConstraintJointVelLimits::PrepareDataForBridging(mtsInterfaceProvided* provided)
{
    /* TEMPLATE: (Make sure there is a matching command in the bridge (e.g. in ros/mts_ros_crtk_ccl_bridge.cpp))
    provided->AddCommandWrite(&{function_to_bridge}, this, Name+cmd_name);
    */

    provided->AddCommandWrite(&mtsConstraintJointVelLimits::set_active, this, Name + "_active");
    provided->AddCommandWrite(&mtsConstraintJointVelLimits::set_lower_limits, this, Name + "_lower_limit");
    provided->AddCommandWrite(&mtsConstraintJointVelLimits::set_upper_limits, this, Name + "_upper_limit");
}
