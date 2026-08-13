#ifndef _mtsVF_RCM_h
#define _mtsVF_RCM_h

#include <cisstCommon/cmnGenericObject.h> 
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstNumerical/nmrLSqLin.h>
#include <sawConstraintController/mtsVFJointPos.h>
#include <sawConstraintController/mtsVFDataRCM.h>
#include <sawConstraintController/mtsVFBase.h>

class mtsVF_RCM : public mtsVFJointPosition
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public:

    mtsVF_RCM() : mtsVFJointPosition(){}
    mtsVF_RCM(const std::string & name, mtsVFDataRCM * data) : mtsVFJointPosition(name,data){
    	H.SetSize(8,6);
        h.SetSize(8);
        H.Zeros();
        h.Zeros();
    }
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double tickTime);

private:

	vct3 ClosestPoint;
    vctDoubleMat H;
    vctDoubleVec h;
    vct3 ur;

	void FillMoveConstraints3D(vctDoubleMat &H, vctDoubleVec &h,
                               vct3 &ur,
                               vct3 direction,
                               vct3 error,
                               double tolerance,
                               int tesselationNumber = 8)
    {    	 
    	H.SetSize(tesselationNumber,6);
        h.SetSize(tesselationNumber);
        H.Zeros();
        h.Zeros();   	
        vct3 unitV_wd;        
        vct3 unitV_pl;        
        vctFixedSizeMatrix<double, 3, 3> R;        
        ZAxisToRotation(direction, R);        
        for (int i = 1; i <= tesselationNumber; i++) {        
            unitV_pl.Assign(cos(i*2*3.14159/tesselationNumber), sin(i*2*3.14159/tesselationNumber), 0);                        
            unitV_wd.ProductOf(R, unitV_pl);                     
            H(i-1, 0)   = -unitV_wd.X();                       
            H(i-1, 1)   = -unitV_wd.Y();                      
            H(i-1, 2)   = -unitV_wd.Z();                    
            h(i-1)      = -tolerance - vctDotProduct(unitV_wd, -error);                     
        }               
        ur.ProductOf(R, error);
    }

    void ZAxisToRotation(vct3 Zaxis, vctFixedSizeMatrix<double, 3, 3> &R)
    {
		vct3 Dx, Dy;
		vct3 YVec3(0., 1., 0.);
		vct3 ZVec3(0., 0., 1.);
		Zaxis.Divide(Zaxis.Norm());
		if ( fabs(vctDotProduct(Zaxis, ZVec3)) > 0.99 ) {
			Dx = (YVec3 % Zaxis);
		} else {
			Dx = (ZVec3 % Zaxis);
		}
		Dx.Divide(Dx.Norm());
		Dy = (Zaxis % Dx);
		Dy.Divide(Dy.Norm());
		R(0, 0) = Dx(0);
		R(1, 0) = Dx(1);
		R(2, 0) = Dx(2);
		R(0, 1) = Dy(0);
		R(1, 1) = Dy(1);
		R(2, 1) = Dy(2);
		R(0, 2) = Zaxis(0);
		R(1, 2) = Zaxis(1);
		R(2, 2) = Zaxis(2);
		bool sanityCheckOkay = true;
		for (int cc = 0; cc < 3; cc++) {
			for (int rr = 0; rr < 3; rr++) {
				if (!CMN_ISFINITE(R(rr, cc))) sanityCheckOkay &= false;
			}
		}
		if (sanityCheckOkay == false) {
			CMN_LOG_CLASS_RUN_ERROR << "#### Dx ####" << std::endl << Dx << "########" << std::endl;
			CMN_LOG_CLASS_RUN_ERROR << "#### Dy ####" << std::endl << Dy << "########" << std::endl;
			CMN_LOG_CLASS_RUN_ERROR << "#### Zaxis ####" << std::endl << Zaxis << "########" << std::endl;
		}
	}

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

    double dist(vctDoubleVec tip, vctDoubleVec closestPoint)
    {
        double x = closestPoint[0] - tip[0];
        double y = closestPoint[1] - tip[1];
        double z = closestPoint[2] - tip[2];
        if(tip*closestPoint > 0)
            return sqrt(x*x + y*y + z*z);
        else
            return -sqrt(x*x + y*y + z*z);
    }

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVF_RCM);

#endif
