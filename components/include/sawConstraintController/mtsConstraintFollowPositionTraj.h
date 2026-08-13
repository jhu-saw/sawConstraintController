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

#ifndef MTSCONSTRAINTFOLLOWPOSITIONTRAJ_H
#define MTSCONSTRAINTFOLLOWPOSITIONTRAJ_H

#include <sawConstraintController/mtsConstraintBase.h>
#include <sawConstraintController/prmKinematicsState.h>

class mtsConstraintFollowPositionTraj : public mtsConstraintBase
{
public:
    mtsConstraintFollowPositionTraj(const Json::Value &constraint_config, const kinematics_map kins_map);
    ~mtsConstraintFollowPositionTraj(){};
    void FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime);
    void PrepareDataForBridging(mtsInterfaceProvided* provided);

    void set_active(const bool &active) // needs to be const ref for provided interface
    {
        Active = active;
        std::cout << Name << " set_active: " << active << std::endl;
    }
    void set_goal_linear_velocity(const double &goal)
    {
        goal_linear_velocity = goal;
        std::cout << Name << " set_goal_linear_velocity: " << goal << std::endl;
    }

    prmKinematicsState *current_kinematics;
    prmKinematicsState *goal_kinematics;
    double goal_linear_velocity;
};

#endif // MTSCONSTRAINTFOLLOWPOSITIONTRAJ_H
