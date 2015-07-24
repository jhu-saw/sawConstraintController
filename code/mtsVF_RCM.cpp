#include <sawConstraintController/mtsVF_RCM.h>

CMN_IMPLEMENT_SERVICES(mtsVF_RCM);

void mtsVF_RCM::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
//std::cout << "Entering RCM" << std::endl;
    mtsVFDataRCM * RCM_Data = (mtsVFDataRCM*)(Data);

    // Constraint-Based RCM
    vct3 Axis = -RCM_Data->TipFrame.Rotation() * vct3(0,0,1);
    ClosestPoint = ClosestLinePoint(vctDoubleVec(RCM_Data->RCM_Point), vctDoubleVec(RCM_Data->TipFrame.Translation()), vctDoubleVec(Axis));
    FillMoveConstraints3D(H,h,ur,Axis,ClosestPoint-RCM_Data->RCM_Point,0.001,RCM_Data->IneqConstraintRows);
    vctDoubleMat RCM_Mat = H*RCM_Data->JacClosest;

    //populate matrix and vector
    IneqConstraintMatrixRef.Assign(RCM_Mat);
    IneqConstraintVectorRef.Assign(h);


    // Objective-Based RCM

    //vct3 RCM_Offset_Base = RCM_Data->TipFrame.Translation(); // TODO temporary use of this variable
    //vct3 dx_translation = RCM_Data->RCM_Point - RCM_Offset_Base;
    //vct3 dx_rotation;
    //vctAxAnRot3 dxRot;
    //vct3 dxRotVec;
    //dxRot.FromNormalized(RCM_Data->TipFrame.Rotation());
    //dxRotVec = dxRot.Axis() * dxRot.Angle();
    //dx_rotation[0] = dxRotVec[0];
    //dx_rotation[1] = dxRotVec[1];
    //dx_rotation[2] = dxRotVec[2];

    //vctDoubleVec dx(6);
    //std::copy(dx_translation.begin(), dx_translation.end(), dx.begin()  );
    //std::copy(dx_rotation.begin()   , dx_rotation.end()   , dx.begin()+3);    

    //vctDoubleMat JacResized = RCM_Data->JacClosest;
    //JacResized.resize(3,7);
    //JacResized.Column(6).SetAll(0.0);
    //ObjectiveMatrixRef.Assign(JacResized);
    //ObjectiveVectorRef.Assign(dx_translation);    

    //std::cout << "H*JacClosest \n" << IneqConstraintMatrixRef << std::endl;
    //std::cout << "h \n" << IneqConstraintVectorRef << std::endl;

    // ConvertRefs(mode,TickTime);
//std::cout << "Leaving RCM" << std::endl;
}
