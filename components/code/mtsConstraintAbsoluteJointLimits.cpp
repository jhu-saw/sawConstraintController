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

Ensures bounded joint angle:
(L-p)*dt < x < (U-p)*dt

| I   0 |   | |    |(L-p) / dt|
|       | . |x| >  |          |
| 0  -I |   | |    |(p-U) / dt|
x: joint velocity
p: current joint position
dt: is timestep
L,U are lower and upper joint limits, respectively
*/

#include <sawConstraintController/mtsConstraintAbsoluteJointLimits.h>

mtsConstraintAbsoluteJointLimits::mtsConstraintAbsoluteJointLimits(const Json::Value &config_param, const kinematics_map kins_map)
    : mtsConstraintBase()
{
    // This is for convenience below
    std::vector<std::string> expected_keys = {"active", "name", "lower_limits", "upper_limits", "kinematics"};

    // Make sure we're actually trying to set the right constraint: Hopefully just sanity check at this point
    if (config_param["type"] != "absolute_joint_limits")
    {
        std::cerr << "[mtsConstraintAbsoluteJointLimits] - incorrect type name: " << config_param["type"] << std::endl;
    }
    // Make sure all expected keys are present
    for (auto &key : expected_keys)
    {
        if (!config_param.isMember(key))
        {
            std::cerr << "[mtsConstraintAbsoluteJointLimits] - expected parameter named: " << key << std::endl;
        }
    }

    // Set all values coming straight from config file
    Active = config_param["active"].asBool();
    Name = config_param["name"].asString();

    cmnDataJSON<vctDoubleVec>::DeSerializeText(upper_limits, config_param["upper_limits"]);
    cmnDataJSON<vctDoubleVec>::DeSerializeText(lower_limits, config_param["lower_limits"]);

    // Systems may slightly overshoot joint limits due to numerical error and/or inertia which breaks the constraint
    // minor deviations allowed but discouraged using slack variables which add a cost to the objective function
    if (config_param.isMember("slack_costs"))
    {
        cmnDataJSON<vctDoubleVec>::DeSerializeText(SlackCosts, config_param["slack_costs"]);
    }
    else
    {
        SlackCosts.SetAll(1.0);
    }

    if (config_param.isMember("slack_limits"))
    {
        cmnDataJSON<vctDoubleVec>::DeSerializeText(SlackLimits, config_param["slack_limits"]);
    }
    else
    {
        auto arbitrary_lims = 0.0025*(upper_limits - lower_limits); // 0.25% of joint range by default
        for (size_t i = 0; i < arbitrary_lims.size(); i++) //Slack limits are [lower, upper]
        {
            SlackLimits(i) = arbitrary_lims(i);
            SlackLimits(i+arbitrary_lims.size()) = arbitrary_lims(i);
        }
    }

    // Set kinematics based on string matching as defined in config file
    std::vector<std::string> expected_kins_names;
    expected_kins_names.push_back(config_param["kinematics"].asString());

    // Make sure all expected kinematics are present in map
    for (auto &name : expected_kins_names)
    {
        if (kins_map.find(name) == kins_map.end())
        {
            std::cerr << "[mtsConstraintAbsoluteJointLimits] - cannot find kinematics in map: " << name << std::endl;
        }
    }

    // Set kinematics
    kinematics = kins_map.at(config_param["kinematics"].asString());

    // Inhernet to constraint
    num_joints = lower_limits.size();
    IneqConstraintRows = 2 * num_joints;
    NumSlacks = 2*num_joints;
}


void mtsConstraintAbsoluteJointLimits::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double TickTime)
{
    // Joint position constraint
    IneqConstraintMatrixRef.SetAll(0.0);
    for (size_t i = 0; i < num_joints; i++)
    {
        IneqConstraintMatrixRef.at(i, i) = TickTime;
        IneqConstraintVectorRef.at(i) = (lower_limits.at(i) - kinematics->JointState->Position()[i]);

        IneqConstraintMatrixRef.at(i+num_joints, i) = -1.0 *  TickTime;
        IneqConstraintVectorRef.at(i+num_joints) = (kinematics->JointState->Position()[i] - upper_limits.at(i));
    }

    // Slack variables
    IneqConstraintMatrixSlackRef.SetAll(0.0);
    IneqConstraintVectorSlackRef.SetAll(0.0);
    ObjectiveMatrixSlackRef.SetAll(0.0);
    for (size_t i = 0; i < num_joints; i++)
    {
        IneqConstraintMatrixSlackRef.at(i, i) = 1.0;
        IneqConstraintMatrixSlackRef.at(i+num_joints, i) = -1.0;
        
        IneqConstraintVectorSlackRef.at(i) = -1.0*SlackLimits.at(i);

        ObjectiveMatrixSlackRef.at(i, i) = SlackCosts.at(i);
        ObjectiveMatrixSlackRef.at(i+num_joints, i) = SlackCosts.at(i+num_joints);
    }
    
}

void mtsConstraintAbsoluteJointLimits::PrepareDataForBridging(mtsInterfaceProvided* provided)
{
    /* TEMPLATE: (Make sure there is a matching command in the bridge (e.g. in ros/mts_ros_crtk_ccl_bridge.cpp))
    provided->AddCommandWrite(&{function_to_bridge}, this, Name+cmd_name);
    */
    provided->AddCommandWrite(&mtsConstraintAbsoluteJointLimits::set_active, this, Name + "_active");
    provided->AddCommandWrite(&mtsConstraintAbsoluteJointLimits::set_lower_limits, this, Name + "_lower_limit");
    provided->AddCommandWrite(&mtsConstraintAbsoluteJointLimits::set_upper_limits, this, Name + "_upper_limit");
}
