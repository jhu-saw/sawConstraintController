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

#ifndef CONSTRAINTCONTROLLERLITE_H
#define CONSTRAINTCONTROLLERLITE_H

#include <sawConstraintController/mtsConstraintBase.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <typeindex>
#include <memory>

class constraintControllerLite: public mtsComponent
{
public:
    typedef std::map<std::string, prmKinematicsState *> kinematics_map;
    constraintControllerLite(int num_joints, const std::string& config_filename, const kinematics_map kins_map = kinematics_map(), bool create_provided_interface=true);
    ~constraintControllerLite(){};
    std::vector<std::shared_ptr<mtsConstraintBase>> constraint_list;
    void SetupConstrainedProblem(double TickTime);
    void PrintProblem();
    nmrConstraintOptimizer Optimizer;
    int num_joints;
    mtsInterfaceProvided* provided;
    bool SolveConstrainedProblem(vctDoubleVec &dq);

private:
    mtsVFBase::CONTROLLERMODE ControllerMode = mtsVFBase::CONTROLLERMODE::JVEL; // We don't use this, but required by optimizer
};

#endif // CONSTRAINTCONTROLLERLITE_H
