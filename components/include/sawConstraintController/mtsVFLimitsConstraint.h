#ifndef SAWCONSTRAINTCONTROLLER_MTSVFLIMITSCONSTRAINT_H
#define SAWCONSTRAINTCONTROLLER_MTSVFLIMITSCONSTRAINT_H

#include <sawConstraintController/mtsVFBase.h>

class mtsVFLimitsConstraint : public mtsVFBase
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsVFLimitsConstraint(): mtsVFBase() {}
    mtsVFLimitsConstraint(const std::string & name, mtsVFDataBase * data):
        mtsVFBase(name, data)
    {}

    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);
private:
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFLimitsConstraint)

#endif //SAWCONSTRAINTCONTROLLER_MTSVFLIMITSCONSTRAINT_H
