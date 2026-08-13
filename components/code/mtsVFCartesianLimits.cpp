/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening
  Created on: 2016

  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawConstraintController/mtsVFCartesianLimits.h>

CMN_IMPLEMENT_SERVICES(mtsVFCartesianLimits)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFCartesianLimits::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{    
    /*
         Fill in refs
         J*dq >= L
         -J*dq >= -U
    */

    CurrentKinematics = Kinematics.at(0);
    vctDoubleMat Jacobian = CurrentKinematics->Jacobian;

    mtsVFDataJointLimits * limitData = reinterpret_cast<mtsVFDataJointLimits*>(Data);

    size_t numLimits = limitData->LowerLimits.size();

    IneqConstraintMatrixRef.SetAll(0.0);

    for(size_t i = 0; i < numLimits; i++)
    {        
      for(size_t j = 0; j < Jacobian.cols(); j++)
      {
        IneqConstraintMatrixRef.at(i,j) = Jacobian(i,j);
        IneqConstraintMatrixRef.at(i+numLimits,j) = -Jacobian(i,j);
      }
      IneqConstraintVectorRef.at(i) = limitData->LowerLimits.at(i);
      IneqConstraintVectorRef.at(i+numLimits) = -limitData->UpperLimits.at(i);
    }

    ConvertRefs(mode,TickTime);
}
