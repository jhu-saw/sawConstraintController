#ifndef MTSVFCYLINDER_H
#define MTSVFCYLINDER_H

#include <cisstCommon/cmnGenericObject.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstNumerical/nmrLSqLin.h>
#include <sawConstraintController/mtsVFJointPos.h>
#include <sawConstraintController/mtsVFDataCylinder.h>
#include <sawConstraintController/mtsVFBase.h>

#include <sawConstraintController/sawConstraintControllerExport.h>

class CISST_EXPORT mtsVFCylinder : public mtsVFBase
{
public:
    mtsVFCylinder(const std::string & name, mtsVFDataBase * data);
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

protected:
    prmKinematicsState * CurrentKinematics;
    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    vctDoubleVec ClosestLinePoint(vctDoubleVec givenPt, vctDoubleVec linePoint, vctDoubleVec lineD)
    {
        vctDoubleVec xcl;
        xcl.SetSize(3);
        double normD = lineD(0) * lineD(0) + lineD(1) * lineD(1) + lineD(2) * lineD(2);
        if (normD > 1e-6) {
        double sqrtNorm = sqrt(normD);
        lineD(0) /= sqrtNorm;
        lineD(1) /= sqrtNorm;
        lineD(2) /= sqrtNorm;
        } else {
        CMN_LOG_CLASS_RUN_ERROR << "FAILURE: " << lineD << std::endl;
        }
        double bx = givenPt(0) - linePoint(0);
        double by = givenPt(1) - linePoint(1);
        double bz = givenPt(2) - linePoint(2);
        double d = bx*lineD(0) + by*lineD(1)+bz*lineD(2);
        if (!CMN_ISFINITE(d)) {
        CMN_LOG_CLASS_RUN_ERROR << "FAILURE: " << lineD << " " << d << std::endl;
        }
        xcl(0) = linePoint(0) + lineD(0)*d;
        xcl(1) = linePoint(1) + lineD(1)*d;
        xcl(2) = linePoint(2) + lineD(2)*d;
        return xcl;
    }
};

#endif // MTSVFCYLINDER_H
