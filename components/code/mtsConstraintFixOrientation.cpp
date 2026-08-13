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
Keep a given frame on the kinematic chain aligned to a specified orientation. 
argmin_x: zeta * || Jw * x + invsk(R_goal*R^-1 - I) ||

zeta is a scaling factor to determine "importance" of constraint
Jw is the 'angular' jacobian at the point of interest, i.e. [angular_velocity = Jw * joint_velocity]
R is the orientation of the frame of interest relative to the base frame, R_goal is what you want this to be
invsk denotes the 'vec' operator, that does the opposite of the 'skew' operator

The frame on the kinematic chain can be specified arbitrarily by specifying the kinematics of the 
last joint that moves the frame of interest as well as any offset from that joint to the frame of interest
The target orientation is represented as a rotation from the base frame to the goal frame
*/

#include <sawConstraintController/mtsConstraintFixOrientation.h>
#include "ccl_helper.h"

mtsConstraintFixOrientation::mtsConstraintFixOrientation(const Json::Value &config_param, const kinematics_map kins_map)
    : mtsConstraintBase()
{
    // This is for convenience below
    std::vector<std::string> expected_keys = {"active", "name", "desired_orientation", "importance_gain", "kinematics_for_last_joint_that_moves_frame_of_interest", "offset_from_kinematics_to_frame_of_interest", "num_joints_system"};

    // Make sure we're actually trying to set the right constraint: Hopefully just sanity check at this point
    if (config_param["type"] != "fix_orientation")
    {
        std::cerr << "[mtsConstraintFixOrientation] - incorrect type name: " << config_param["type"] << std::endl;
    }
    // Make sure all expected keys are present
    for (auto &key : expected_keys)
    {
        if (!config_param.isMember(key))
        {
            std::cerr << "[mtsConstraintFixOrientation] - expected parameter named: " << key << std::endl;
        }
    }

    // Set all values coming straight from config file (All should have this)
    Active = config_param["active"].asBool();
    Name = config_param["name"].asString();

    // Custom / implemntation-specific settings
    num_joints_system = config_param["num_joints_system"].asInt();
    importance_gain = config_param["importance_gain"].asDouble();
    cmnDataJSON<vctFrm3>::DeSerializeText(offset_from_kinematics_to_frame_of_interest, config_param["offset_from_kinematics_to_frame_of_interest"]);
    cmnDataJSON<vctRot3>::DeSerializeText(desired_orientation, config_param["desired_orientation"]);

    // Set kinematics based on string matching as defined in config file
    std::vector<std::string> expected_kins_names;
    expected_kins_names.push_back(config_param["kinematics_for_last_joint_that_moves_frame_of_interest"].asString());

    // Make sure all expected kinematics are present in map
    for (auto &name : expected_kins_names)
    {
        if (kins_map.find(name) == kins_map.end())
        {
            std::cerr << "[mtsConstraintFixOrientation] - cannot find kinematics in map: " << name << std::endl;
        }
    }

    // Set kinematics
    kinematics_for_last_joint_that_moves_frame_of_interest = kins_map.at(config_param["kinematics_for_last_joint_that_moves_frame_of_interest"].asString());

    // Inhernet to constraint
    ObjectiveRows = 3;
}


void mtsConstraintFixOrientation::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double CMN_UNUSED(TickTime))
{
    // Calculate pose and jacobian at frame of interest (the frame you are trying to set the orientation of)
    // Note this jacobian is for all joints in the full system, columns associated with more distal joints are set to zero
    vctFrm3 frame_of_interest = kinematics_for_last_joint_that_moves_frame_of_interest->Frame * offset_from_kinematics_to_frame_of_interest;
    vctDoubleMat jacobian_of_interest = ccl_helper::GetJacobianAtTransformedBodyReference(kinematics_for_last_joint_that_moves_frame_of_interest->Jacobian,
        offset_from_kinematics_to_frame_of_interest,
        kinematics_for_last_joint_that_moves_frame_of_interest->Frame);
    vctDoubleMat system_angular_jacobian_at_frame_of_reference;
    ccl_helper::ExtractAngularJacobianToArbitrarySize(jacobian_of_interest, system_angular_jacobian_at_frame_of_reference, num_joints_system);

    auto& current_orientation = frame_of_interest.Rotation();
    vct3 vect;
    ccl_helper::InvSkew3x3MatTo3Vec((desired_orientation * current_orientation.Inverse())-vct3x3::Eye(), vect);
    // Fill in A matrix
    ObjectiveMatrixRef.Assign(importance_gain * system_angular_jacobian_at_frame_of_reference);
    // Fill in b vector
    ObjectiveVectorRef.Assign(importance_gain * vect);
}

void mtsConstraintFixOrientation::PrepareDataForBridging(mtsInterfaceProvided* provided)
{
    /* TEMPLATE: (Make sure there is a matching command in the bridge (e.g. in ros/mts_ros_crtk_ccl_bridge.cpp))
    provided->AddCommandWrite(&{function_to_bridge}, this, Name+cmd_name);
    */
    provided->AddCommandWrite(&mtsConstraintFixOrientation::set_active, this, Name + "_active");
    provided->AddCommandWrite(&mtsConstraintFixOrientation::set_importance_gain, this, Name + "_importance_gain");
    provided->AddCommandWrite(&mtsConstraintFixOrientation::set_desired_orientation, this, Name + "_desired_orientation");

}
