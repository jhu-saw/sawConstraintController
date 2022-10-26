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


#include "ccl_helper.h"

namespace ccl_helper
{
    vctDoubleMat ChangeJacobianByOffset(const vctDoubleMat &jacobian, const vctFrm3 &offset, const vctFrm3 &self_fk)
    {
        // Here Jacobian is defined as in Spong, This is the so-called "hybrid-velocity" Jacobian as defined in MLS Mathematical Introduction to Robot...

        vctDoubleMat jac_out = jacobian;
        vctRot3 self_FK_R = self_fk.Rotation();

        // vctRot3 R = offset.Rotation();

        for (size_t i = 0; i < jac_out.cols(); i++)
        {
            vctDouble3 v_start;
            vctDoubleVec column = jac_out.Column(i);
            std::copy(column.begin(), column.begin() + 3, v_start.begin());
            vctDouble3 w_start;
            std::copy(column.begin() + 3, column.end(), w_start.begin());

            vctDouble3x3 w_start_hat;
            ccl_helper::Skew3VecTo3x3Mat(w_start, w_start_hat);
            vctDouble3 v_target = v_start + w_start_hat * (self_FK_R * offset.Translation()); // put offset translation into origin/base coords
            vctDouble3 w_target = w_start;
            std::copy(v_target.begin(), v_target.end(), jac_out.Column(i).begin());
            std::copy(w_target.begin(), w_target.end(), jac_out.Column(i).begin() + 3);
        }
        return jac_out;
    }

    /**
     * @brief Mathematical skew or 'hat' operator taking vector to matrix (e.g. rotation vector axis*angle to element of so(3))
     *
     * @param vector Vector in R3
     * @param mat Resulting skew / hat matrix
     */
    void Skew3VecTo3x3Mat(const vct3 &vector, vctDouble3x3 &mat)
    {
        mat.SetAll(0.0);
        mat(2, 1) = vector(0); // x
        mat(0, 2) = vector(1); // y
        mat(1, 0) = vector(2); // z

        mat(1, 2) = -mat(2, 1); //-x
        mat(2, 0) = -mat(0, 2); //-y
        mat(0, 1) = -mat(1, 0); //-z
    }

    void Skew3VecTo3x3Mat(const vct3 &vector, vctDoubleMat &mat)
    {
        mat.resize(3, 3);
        mat.SetAll(0.0);
        mat(2, 1) = vector(0); // x
        mat(0, 2) = vector(1); // y
        mat(1, 0) = vector(2); // z

        mat(1, 2) = -mat(2, 1); //-x
        mat(2, 0) = -mat(0, 2); //-y
        mat(0, 1) = -mat(1, 0); //-z
    }

    void InvSkew3x3MatTo3Vec(const vctDouble3x3 &mat, vct3 &vector)
    {
        // could check to be sure diags are zero here
        vector(0) = mat(2, 1); // x
        vector(1) = mat(0, 2); // y
        vector(2) = mat(1, 0); // z
    }

    void ExtractAngularJacobianToArbitrarySize(vctDoubleMat &full_jacobian, vctDoubleMat &out_jacobian, int out_cols)
    {
        out_jacobian.resize(3, out_cols);
        out_jacobian.SetAll(0.0);
        out_jacobian.Ref(3, full_jacobian.cols()) = full_jacobian.Ref(3, full_jacobian.cols(), 3, 0);
    }

    void ExtractPositionalJacobianToArbitrarySize(vctDoubleMat &full_jacobian, vctDoubleMat &out_jacobian, int out_cols)
    {
        out_jacobian.resize(3, out_cols);
        out_jacobian.SetAll(0.0);
        out_jacobian.Ref(3, full_jacobian.cols()) = full_jacobian.Ref(3, full_jacobian.cols());
    }
}
