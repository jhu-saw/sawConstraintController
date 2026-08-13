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


#ifndef constraintControllerLite_HELPER_H
#define constraintControllerLite_HELPER_H

#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstVector.h>

// some helpers used in constraint controller lite implementation. They may be replacable by existing functionality from elsewhere
namespace ccl_helper
{
    vctDoubleMat GetJacobianAtTransformedBodyReference(const vctDoubleMat &jacobian, const vctFrm3 &transform, const vctFrm3 &self_fk);
    void Skew3VecTo3x3Mat(const vct3 &vector, vctDouble3x3 &mat);
    void Skew3VecTo3x3Mat(const vct3 &vector, vctDoubleMat &mat);
    void InvSkew3x3MatTo3Vec(const vctDouble3x3 &mat, vct3 &vector);
    void ExtractAngularJacobianToArbitrarySize(const vctDoubleMat &full_jacobian, vctDoubleMat &out_jacobian, const int out_cols);
    void ExtractPositionalJacobianToArbitrarySize(const vctDoubleMat &full_jacobian, vctDoubleMat &out_jacobian, const int out_cols);
}
#endif // constraintControllerLite_HELPER_H
