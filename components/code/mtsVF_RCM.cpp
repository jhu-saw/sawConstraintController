#include <sawConstraintController/mtsVF_RCM.h>

CMN_IMPLEMENT_SERVICES(mtsVF_RCM);

void mtsVF_RCM::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{

    mtsVFDataRCM * RCM_Data = (mtsVFDataRCM*)(Data);

    // Constraint-Based RCM
    vct3 Axis = -RCM_Data->TipFrame.Rotation() * vct3(0,0,1);
    ClosestPoint = ClosestLinePoint(vctDoubleVec(RCM_Data->RCM_Point), vctDoubleVec(RCM_Data->TipFrame.Translation()), vctDoubleVec(Axis));
    FillMoveConstraints3D(H,h,ur,Axis,ClosestPoint-RCM_Data->RCM_Point,0.001,RCM_Data->IneqConstraintRows);
    vctDoubleMat RCM_Mat = H*RCM_Data->JacClosest;

    //populate matrix and vector
    IneqConstraintMatrixRef.Assign(RCM_Mat);
    IneqConstraintVectorRef.Assign(h);

    // ConvertRefs(mode,TickTime);

}
