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
    /**
     * @brief Change the location of the 'body'/'tool' reference frame of the MLS 'Hybrid'/Spong Manipulator Jacobian using a transformation from its current to new reference frames
     *
     * @param jacobian Matrix mapping joint velocities to cartesian velocities
     * @param transform Transform mapping from current to new base reference frames
     * @return vctDoubleMat Jacobian in new reference frame
     * @details The Jacobian used here is the what Spong calls the 'Jacobian' and what Mathematical Introduction to Robotic Manipulation (Murray, Li, Sastry 1994) calls the 'Hybrid Jacobian'.
     */
    vctDoubleMat GetJacobianAtTransformedBodyReference(const vctDoubleMat &jacobian, const vctFrm3 &offset, const vctFrm3 &self_fk)
    {
        vctDoubleMat jac_out = jacobian;
        vctRot3 self_FK_R = self_fk.Rotation();

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

    /**
     * @brief Mathematical skew or 'hat' operator taking vector to matrix (e.g. rotation vector axis*angle to element of so(3))
     *
     * @param vector Vector in R3
     * @param mat Resulting skew / hat matrix
     */ 
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

    /**
     * @brief Inverse of the skew or 'hat' operator taking matrix to vector (e.g. element of so(3) to axis angle rotation vector)
     *
     * @param mat Resulting skew / hat matrix
     * @param vector Vector in R3
     */
    void InvSkew3x3MatTo3Vec(const vctDouble3x3 &mat, vct3 &vector)
    {
        // could check to be sure diags are zero here
        vector(0) = mat(2, 1); // x
        vector(1) = mat(0, 2); // y
        vector(2) = mat(1, 0); // z
    }

    /**
     * @brief Take a 6xN Jacobian and extract last 3 rows (angular velocity part), and zero pad at right to arbitrary size
     *
     * @param full_jacobian 6xN Jacobian input
     * @param out_jacobian 3xN Jacobian output
     * @param out_cols Number of columns in output Jacobian (additional will be zero padded at right)
     */
    void ExtractAngularJacobianToArbitrarySize(const vctDoubleMat &full_jacobian, vctDoubleMat &out_jacobian, const int out_cols)
    {
        out_jacobian.resize(3, out_cols);
        out_jacobian.SetAll(0.0);
        out_jacobian.Ref(3, full_jacobian.cols()) = full_jacobian.Ref(3, full_jacobian.cols(), 3, 0);
    }

    /**
     * @brief Take a 6xN Jacobian and extract first 3 rows (linear velocity part), and zero pad at right to arbitrary size
     *
     * @param full_jacobian 6xN Jacobian input
     * @param out_jacobian 3xN Jacobian output
     * @param out_cols Number of columns in output Jacobian (additional will be zero padded at right)
     */
    void ExtractPositionalJacobianToArbitrarySize(const vctDoubleMat &full_jacobian, vctDoubleMat &out_jacobian, const int out_cols)
    {
        out_jacobian.resize(3, out_cols);
        out_jacobian.SetAll(0.0);
        out_jacobian.Ref(3, full_jacobian.cols()) = full_jacobian.Ref(3, full_jacobian.cols());
    }
}
