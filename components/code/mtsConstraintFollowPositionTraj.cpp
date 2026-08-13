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
Adds objective term (i.e. fills in 'A' and 'b' terms) to the constained optimization problem on joint velocity x 
argmin_x: ||Ax+b||, s.t. Cx>d, Ex=f

Penalizes joint velocities that will not:
Drive the position of the specified kinematics to a specified goal point in 3D space. 
argmin_x: || Jp*x + v_des ||
Jp is the 'positional' jacobian at the point of interest, i.e. [linear_velocity = Jp * joint_velocity]
v_des is the desired incremental velocity of the end effector over the next time step

Used sequentially, this allows for trajectory following
*/

#include <sawConstraintController/mtsConstraintFollowPositionTraj.h>

mtsConstraintFollowPositionTraj::mtsConstraintFollowPositionTraj(const Json::Value &config_param, const kinematics_map kins_map)
    : mtsConstraintBase()
{
    // This is for convenience below
    std::vector<std::string> expected_keys = {"active", "name", "goal_linear_velocity_m/s", "current_kinematics", "goal_kinematics"};

    // Make sure we're actually trying to set the right constraint: Hopefully just sanity check at this point
    if (config_param["type"] != "follow_position_trajectory")
    {
        std::cerr << "[mtsConstraintFollowPositionTraj] - incorrect type name: " << config_param["type"] << std::endl;
    }
    // Make sure all expected keys are present
    for (auto &key : expected_keys)
    {
        if (!config_param.isMember(key))
        {
            std::cerr << "[mtsConstraintFollowPositionTraj] - expected parameter named: " << key << std::endl;
        }
    }

    // Set all values coming straight from config file
    Active = config_param["active"].asBool();
    Name = config_param["name"].asString();
    goal_linear_velocity = config_param["goal_linear_velocity_m/s"].asDouble();

    // Set kinematics based on string matching as defined in config file
    std::vector<std::string> expected_kins_names;
    expected_kins_names.push_back(config_param["current_kinematics"].asString());
    expected_kins_names.push_back(config_param["goal_kinematics"].asString());

    // Make sure all expected kinematics are present in map
    for (auto &name : expected_kins_names)
    {
        if (kins_map.find(name) == kins_map.end())
        {
            std::cerr << "[mtsConstraintFollowPositionTraj] - cannot find kinematics in map: " << name << std::endl;
        }
    }

    // Set kinematics
    current_kinematics = kins_map.at(config_param["current_kinematics"].asString());
    goal_kinematics = kins_map.at(config_param["goal_kinematics"].asString());

    // Inhernet to constraint
    ObjectiveRows = 3;

}


void mtsConstraintFollowPositionTraj::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double TickTime)
{
    double dt = TickTime;
    vct3 p_to_goal = goal_kinematics->Frame.Translation() - current_kinematics->Frame.Translation();

    // Performs linear interpolation in case far from the goal point
    double T = p_to_goal.Norm() / goal_linear_velocity;  // time to goal at desired linear velocity (s)
    double lambda = std::max(0.0, (T - dt) / T);
    vct3 p_interp_goal = goal_kinematics->Frame.Translation() - lambda * (p_to_goal);

    vct3 p_move = p_interp_goal - current_kinematics->Frame.Translation();
    if (ObjectiveRows == 3)
    {
        // Fill A matrix
        ObjectiveMatrixRef.Assign(current_kinematics->Jacobian.Ref(3, current_kinematics->Jacobian.cols()));  // extracts position rows of jacobian
        // Fill b vector
        ObjectiveVectorRef.Assign(p_move / dt);  // this is the desired linear velocity
    }
    else
    {
        std::cerr << "[mtsConstraintFollowPositionTraj]: Incorrect number of Objective rows" << std::endl;
    }
}

void mtsConstraintFollowPositionTraj::PrepareDataForBridging(mtsInterfaceProvided* provided)
{
    /* TEMPLATE: (Make sure there is a matching command in the bridge (e.g. in ros/mts_ros_crtk_ccl_bridge.cpp))
    provided->AddCommandWrite(&{function_to_bridge}, this, Name+cmd_name);
    */
    provided->AddCommandWrite(&mtsConstraintFollowPositionTraj::set_active, this, Name + "_active");
    provided->AddCommandWrite(&mtsConstraintFollowPositionTraj::set_goal_linear_velocity, this, Name + "_goal_linear_velocity");

}
