/*
  Author(s):  Max Zhaoshuo Li
  Created on: 2019-10-21

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _simpleCoop_h
#define _simpleCoop_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

// vf
#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFSensorCompliance.h>
#include <sawConstraintController/mtsVFPlane.h>
#include <sawConstraintController/mtsVFLimitsConstraint.h>
#include <sawConstraintController/mtsVFCylinder.h>

// vf data
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/mtsVFDataSensorCompliance.h>
#include <sawConstraintController/mtsVFDataPlane.h>
#include <sawConstraintController/mtsVFDataJointLimits.h>
#include <sawConstraintController/mtsVFDataCylinder.h>

class simpleCoop: public mtsTaskPeriodic {
protected:
    // internal method to configure this component
    void Init();
    void SetupRobot();

    //! Set up the behaviours for the robot
    /*!
     * \brief setupVFBehaviour
     * User needs to add the desired behaviour inside this function. This library only means to provide a receipe instead of a full API.
     * For details of how to define behaviours, please check the code in sawConstraintOptimizer.
     */
    void SetupVFBehaviour();

    //! Run funtion for constraint behaviour
    /*!
     * \brief runBehaviour
     * \param dq the incremental joint value
     * \return optimizer STATUS,
     * 0  Both equality and inequality constraints are compatible and have been satisfied.
     * 1  Equality constraints are contradictory. A generalized inverse solution of EX=F was used to minimize the residual vector length F-EX. In this sense, the solution is still meaningful.
     * 2  Inequality constraints are contradictory.
     * 3  Both equality and inequality constraints are contradictory.
     * 4  Input has a NaN or INF
     */
    nmrConstraintOptimizer::STATUS RunBehaviour(vctDoubleVec & dq);

    // robot specific variables
    void ForwardKinematics(vctDoubleVec& jointPosition);
    vctDoubleVec mJointPosition;
    vctDoubleMat mJacobian;
    vctFrm4x4 mCartesianPosition;
    prmPositionCartesianGet mMeasuredCartesianPosition; // for ros publication

    int mNumOutput;
    int mNumJoints;
    int mNumFTDoF;

    // constraint controller
    mtsVFController *mController;   //! Constraint controller that controls the behaviours
    prmKinematicsState mMeasuredKinematics; //! Current kinematics information (frame, jacobian) from the robot
    prmSensorState mGoalForceValues; //! Current force sensor information from the robot
    prmStateJoint mMeasuredJoint; //! Current joint information (joint values) from the robot
    mtsVFDataSensorCompliance mCoopObjective; //! Compliance control behaviour
    mtsVFDataPlane mPlaneConstraint;
    mtsVFDataJointLimits mJointLimitsConstraint;
    mtsVFDataCylinder mCylindricalConstraint;

    //! Update the numerical solver of the robot kinematics values
    /*!
     * \brief updateOptimizerKinematics
     * Update the necessary kinematics information for the optimizer
     */
    void UpdateOptimizerKinematics();
    //! Update the numerical solver of the sensor values
    /*!
     * \brief updateOptimizerSensor
     * Update the necessary sensor information for the optimizer
     */
    void UpdateOptimizerSensor();

    void ServoCartesianForce(const mtsDoubleVec & newGoal);

public:
    //! Constructor
    simpleCoop(const std::string & componentName, const double periodInSeconds);
    ~simpleCoop() {}

    //! Run loop for the robot task
    void Run();        // performed periodically
};

#endif // _simpleCoop_h
