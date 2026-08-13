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

#ifndef mtsConstraintStayOnAxis_H
#define mtsConstraintStayOnAxis_H

#include <sawConstraintController/mtsConstraintBase.h>
#include <sawConstraintController/prmKinematicsState.h>

class mtsConstraintStayOnAxis : public mtsConstraintBase
{
public:
    mtsConstraintStayOnAxis(const Json::Value &constraint_config, const kinematics_map kins_map);
    ~mtsConstraintStayOnAxis(){};
    void FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime);
    void PrepareDataForBridging(mtsInterfaceProvided* provided);

    void set_active(const bool &active) // needs to be const ref for provided interface
    {
        Active = active;
        std::cout << Name << " set_active: " << active << std::endl;
    }
    void set_importance_gain(const double &gain)
    {
        importance_gain = gain;
        std::cout << Name << " set_importance_gain: " << gain << std::endl;
    }
    void set_desired_axis(const vct3 &v)
    {
        desired_axis = v;
        std::cout << Name << " set_desired_axis: " << v << std::endl;
    }
    void set_point_on_desired_axis(const vct3 &p)
    {
        point_on_desired_axis = p;
        std::cout << Name << " point_on_desired_axis: " << p << std::endl;
    }

    prmKinematicsState *kinematics_for_last_joint_that_moves_frame_of_interest;
    vctFrm3 offset_from_kinematics_to_frame_of_interest;
    vct3 desired_axis;
    vct3 point_on_desired_axis;
    double importance_gain;
    int num_joints_system;

};

#endif // mtsConstraintStayOnAxis_H
