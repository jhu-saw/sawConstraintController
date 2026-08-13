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
A re-implementation of the sawConstraintController that aims to expose the most useful functionality, while simplifying the creation and use of
constraints. This is built on top of the existing framework, so it does inherit a lot of the prior terminology and classes under the hood.
Several prior features have been removed for simplicity (e.g. ControllerMode and the separation of VFData and VF types).

While the addition of too many features was one of the difficulties I found associated with the previous implementation, I have taken the risk to
add a few features that I find convenient. Most of them should be optional. These include:
- Allow for the configuration of constraints via a JSON file
- Allow for the specification of members that are exposed to ROS via provided interfaces (e.g. to set if a constraint is active, or some associated value)

You add terms to the following constrained optimization problem by creating a derived version of mtsConstraintBase. Examples are provided for the cpp and h files

Essentially, you define a constained optimization problem on joint velocity x
argmin_x: ||Ax+b||, s.t. Cx>d, Ex=f

Potential features that could be added
- Some way to not have to add constraint types in this cpp when you make new ones. It is a small addition, but still a bit annoying
  essentially, there is likely a 'factory' of sorts that could be used to simplify the process of adding new constraint implementations
- Templates for cpp and h files
*/

#include <sawConstraintController/constraintControllerLite.h>
#include <sawConstraintController/mtsConstraintFollowPositionTraj.h>
#include <sawConstraintController/mtsConstraintAbsoluteJointLimits.h>
#include <sawConstraintController/mtsConstraintJointVelLimits.h>
#include <sawConstraintController/mtsConstraintFixOrientation.h>
#include <sawConstraintController/mtsConstraintStayOnAxis.h>

/* TEMPLATE:
#include <sawConstraintController/{CLASS_TYPE}.h>
*/

#include <cisstMultiTask/mtsInterfaceProvided.h>

constraintControllerLite::constraintControllerLite(int num_joints, const std::string &config_filename, const kinematics_map kins_map, bool create_provided_interface )
    : Optimizer(num_joints), num_joints(num_joints)
{
    Name = "controller";
    std::ifstream jsonStream;
    jsonStream.open(config_filename.c_str());
    Json::Value jsonConfig, jsonConstraints;
    Json::Reader jsonReader;
    jsonReader.parse(jsonStream, jsonConfig);
    jsonConstraints = jsonConfig["constraints"];
    jsonStream.close();

    provided = this->AddInterfaceProvided("ConstraintConfig");

    for (auto &jsonConstraint : jsonConstraints)
    {
        std::shared_ptr<mtsConstraintBase> constraint;
        //TODO: find a way to just loop through a list or map of these
        if (jsonConstraint["type"] == "follow_position_trajectory")
        {
            constraint = std::make_shared<mtsConstraintFollowPositionTraj>(jsonConstraint, kins_map);
        }
        else if (jsonConstraint["type"] == "absolute_joint_limits")
        {
            constraint = std::make_shared<mtsConstraintAbsoluteJointLimits>(jsonConstraint, kins_map);
        }
        else if (jsonConstraint["type"] == "joint_velocity_limits")
        {
            constraint = std::make_shared<mtsConstraintJointVelLimits>(jsonConstraint, kins_map);
        }
        else if (jsonConstraint["type"] == "fix_orientation")
        {
            constraint = std::make_shared<mtsConstraintFixOrientation>(jsonConstraint, kins_map);
        }
        else if (jsonConstraint["type"] == "stay_on_axis")
        {
            constraint = std::make_shared<mtsConstraintStayOnAxis>(jsonConstraint, kins_map);
        }
        /* TEMPLATE:
        else if (jsonConstraint["type"] == "{CONFIG_NAME}")
        {
            constraint = std::make_shared<{CLASS_TYPE}>(jsonConstraint, kins_map);
        }
        */

        else
        {
            std::cout << "[constraintControllerLite]: no configuration found for constraint of type: " << jsonConstraint["type"] << std::endl;
            continue;
        }

        // Add configured constraint
        constraint_list.push_back(constraint);

        // Let the constraint define what's needed for bridging (e.g. cisst-ros bridge), this is to allow easier isolation of implementation-specific items
        if (create_provided_interface)
        {
            constraint->PrepareDataForBridging(provided);
        }
    }
}


void constraintControllerLite::SetupConstrainedProblem(double TickTime)
{
    // Loop through once to find the space needed in the tableau
    // (this is most important when using slack variables, and allows for turning constraints off/on in realtime)
    Optimizer.ResetIndices();

    for (auto &constraint : constraint_list)
    {
        if (constraint->Active)
        {
            Optimizer.ReserveSpace(constraint->ObjectiveRows, constraint->IneqConstraintRows, constraint->EqConstraintRows, constraint->NumSlacks);
        }
    }
    Optimizer.Allocate();

    // Loop through again to fill in tableau
    Optimizer.ResetIndices();

    for (auto &constraint : constraint_list)
    {
        if (constraint->Active)
        {
            // updates the tableau references
            Optimizer.SetRefs(constraint->ObjectiveRows, constraint->IneqConstraintRows, constraint->EqConstraintRows,
                             constraint->NumSlacks, constraint->ObjectiveMatrixRef, constraint->ObjectiveMatrixSlackRef, constraint->ObjectiveVectorRef,
                             constraint->IneqConstraintMatrixRef, constraint->IneqConstraintMatrixSlackRef, constraint->IneqConstraintVectorRef, constraint->IneqConstraintVectorSlackRef,
                             constraint->EqConstraintMatrixRef, constraint->EqConstraintVectorRef);

            // fills in the tableau (derived classes of mtsConstraintBase override this method with their own logic), some combination of A,b,C,d,E,f are set here
            constraint->FillInTableauRefs(ControllerMode, TickTime);
        }
    }
}

bool constraintControllerLite::SolveConstrainedProblem(vctDoubleVec &dq)
{
    nmrConstraintOptimizer::STATUS status = Optimizer.Solve(dq);
    return status == nmrConstraintOptimizer::NMR_OK;
}

void constraintControllerLite::PrintProblem()
{
  std::cout << "\n\nObjectiveMatrix:\n" << Optimizer.GetObjectiveMatrix() << std::endl;
  std::cout << "\nObjectiveVector:\n" << Optimizer.GetObjectiveVector() << std::endl;
  std::cout << "\nEqualityConstraintMatrix:\n" << Optimizer.GetEqConstraintMatrix() << std::endl;
  std::cout << "\nEqualityConstraintVector:\n" << Optimizer.GetEqConstraintVector() << std::endl;
  std::cout << "\nInequalityConstraintMatrix:\n" << Optimizer.GetIneqConstraintMatrix() << std::endl;
  std::cout << "\nInequalityConstraintVector:\n" << Optimizer.GetIneqConstraintVector() << std::endl;
}