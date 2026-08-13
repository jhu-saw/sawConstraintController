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

#ifndef mtsConstraintBase_H
#define mtsConstraintBase_H

#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <memory>
#include <typeindex>

/* For use with the 'lite' version of the consraint controller */

class mtsConstraintBase : public mtsVFDataBase, public mtsVFBase
{
public:
    typedef std::map<std::string, prmKinematicsState *> kinematics_map;
    mtsConstraintBase(){};
    virtual ~mtsConstraintBase(){};
    virtual void PrepareDataForBridging(mtsInterfaceProvided* CMN_UNUSED(provided)) {};

    using mtsVFBase::Name; // clarify ambiguity as both have this member

private:
    virtual void ConvertRefs(const mtsVFBase::CONTROLLERMODE CMN_UNUSED(mode),
                             const double CMN_UNUSED(TickTime)) {}; // Required by mtsVFBase, but won't be using

protected:
};

#endif // mtsConstraintBase_H
