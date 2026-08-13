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
keep a specified point on the kinematic chain on a line/axis defined in 3D space. 

argmin_x: zeta * || [-sk(v_axis) * Jp] x + sk(v_axis)*(p_curr-p_axis) ||
zeta is a scaling factor to determine "importance" of constraint
Jp is the 'positional' jacobian at the point of interest, i.e. [linear_velocity = Jp * joint_velocity]
p_curr is the current position of the point in 3D space
p_axis and v_axis define a line/axis in 3D space using a point p and direction vector v

The point on the kinematic chain can be specified arbitrarily by specifying the kinematics of the 
last joint that moves the point of interest as well as any offset from that joint to the point of interest

Minimizes the cross product between the direction vector of the axis you are trying to stay on and
the vector between a point on that axis and the new position of the point after the input joint values
*/

#include <sawConstraintController/mtsConstraintStayOnAxis.h>
#include "ccl_helper.h"

mtsConstraintStayOnAxis::mtsConstraintStayOnAxis(const Json::Value &config_param, const kinematics_map kins_map)
    : mtsConstraintBase()
{
    // This is for convenience below
    std::vector<std::string> expected_keys = {"active", "name", "desired_axis", "importance_gain", "point_on_desired_axis", "kinematics_for_last_joint_that_moves_frame_of_interest", "offset_from_kinematics_to_frame_of_interest", "num_joints_system"};

    // Make sure we're actually trying to set the right constraint: Hopefully just sanity check at this point
    if (config_param["type"] != "stay_on_axis")
    {
        std::cerr << "[mtsConstraintStayOnAxis] - incorrect type name: " << config_param["type"] << std::endl;
    }
    // Make sure all expected keys are present
    for (auto &key : expected_keys)
    {
        if (!config_param.isMember(key))
        {
            std::cerr << "[mtsConstraintStayOnAxis] - expected parameter named: " << key << std::endl;
        }
    }

    // Set all values coming straight from config file (All should have this)
    Active = config_param["active"].asBool();
    Name = config_param["name"].asString();

    // Custom / implemntation-specific settings
    num_joints_system = config_param["num_joints_system"].asInt();
    importance_gain = config_param["importance_gain"].asDouble();
    cmnDataJSON<vctFrm3>::DeSerializeText(offset_from_kinematics_to_frame_of_interest, config_param["offset_from_kinematics_to_frame_of_interest"]);
    cmnDataJSON<vct3>::DeSerializeText(desired_axis, config_param["desired_axis"]);
    cmnDataJSON<vct3>::DeSerializeText(point_on_desired_axis, config_param["point_on_desired_axis"]);

    // Set kinematics based on string matching as defined in config file
    std::vector<std::string> expected_kins_names;
    expected_kins_names.push_back(config_param["kinematics_for_last_joint_that_moves_frame_of_interest"].asString());

    // Make sure all expected kinematics are present in map
    for (auto &name : expected_kins_names)
    {
        if (kins_map.find(name) == kins_map.end())
        {
            std::cerr << "[mtsConstraintStayOnAxis] - cannot find kinematics in map: " << name << std::endl;
        }
    }

    // Set kinematics
    kinematics_for_last_joint_that_moves_frame_of_interest = kins_map.at(config_param["kinematics_for_last_joint_that_moves_frame_of_interest"].asString());

    // Inhernet to constraint
    ObjectiveRows = 3;

}


void mtsConstraintStayOnAxis::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double CMN_UNUSED(TickTime))
{
    // Calculate pose and jacobian at point of interest (the point you are trying to get to stay on the axis)
    // Note this jacobian is for all joints in the full system, columns associated with more distal joints are set to zero
    vctFrm3 frame_of_interest = kinematics_for_last_joint_that_moves_frame_of_interest->Frame * offset_from_kinematics_to_frame_of_interest;
    vctDoubleMat jacobian_of_interest = ccl_helper::GetJacobianAtTransformedBodyReference(kinematics_for_last_joint_that_moves_frame_of_interest->Jacobian,
        offset_from_kinematics_to_frame_of_interest,
        kinematics_for_last_joint_that_moves_frame_of_interest->Frame);
    vctDoubleMat system_positional_jacobian_at_frame_of_reference;
    ccl_helper::ExtractPositionalJacobianToArbitrarySize(jacobian_of_interest, system_positional_jacobian_at_frame_of_reference, num_joints_system);

    auto& current_pos = frame_of_interest.Translation();
    vctDoubleMat skew_ax;
    ccl_helper::Skew3VecTo3x3Mat(desired_axis, skew_ax);
    // Fill in A matrix
    ObjectiveMatrixRef.Assign(-1.0 * importance_gain * skew_ax * system_positional_jacobian_at_frame_of_reference);
    // Fill in b vector
    ObjectiveVectorRef.Assign(importance_gain * skew_ax * vctDoubleVec(current_pos - point_on_desired_axis));
}

void mtsConstraintStayOnAxis::PrepareDataForBridging(mtsInterfaceProvided* provided)
{
    /* TEMPLATE: (Make sure there is a matching command in the bridge (e.g. in ros/mts_ros_crtk_ccl_bridge.cpp))
    provided->AddCommandWrite(&{function_to_bridge}, this, Name+cmd_name);
    */

    provided->AddCommandWrite(&mtsConstraintStayOnAxis::set_active, this, Name + "_active");
    provided->AddCommandWrite(&mtsConstraintStayOnAxis::set_importance_gain, this, Name + "_importance_gain");
    provided->AddCommandWrite(&mtsConstraintStayOnAxis::set_desired_axis, this, Name + "_desired_axis");
    provided->AddCommandWrite(&mtsConstraintStayOnAxis::set_point_on_desired_axis, this, Name + "_point_on_desired_axis");
}
