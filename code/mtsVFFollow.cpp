/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening
  Created on: 2014

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawConstraintController/mtsVFDaVinciFollow.h>

CMN_IMPLEMENT_SERVICES(mtsVFFollow)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFFollow::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    // fill in refs
    // min || I*dx - d ||
    // I is the identity matrix
    // d is the objective vector, which stores the desired cartesian position for the slave

    //now set reference to the left hand side of the above equation, the identity matrix
    ObjectiveMatrixRef.Diagonal().SetAll(1.0);

    //set the reference to the right hand side of the above equation (d)
    ObjectiveVectorRef.Assign(Data->ObjectiveVector);

    ConvertRefs(mode,TickTime);
}
