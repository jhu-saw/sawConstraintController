#ifndef _simpleRobot_h
#define _simpleRobot_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFSensorCompliance.h>
#include <sawConstraintController/mtsVFPlane.h>
#include <sawConstraintController/mtsVFLimitsConstraint.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/mtsVFDataSensorCompliance.h>
#include <sawConstraintController/mtsVFDataPlane.h>
#include <sawConstraintController/mtsVFDataJointLimits.h>

class simpleRobot: public mtsTaskPeriodic {
protected:
    // internal method to configure this component
    void init();
    void setupRobot();
    void setupVF();
    nmrConstraintOptimizer::STATUS runBehaviour(vctDoubleVec & dq);

    // robot specific variables
    void forwardKinematics(vctDoubleVec& jointPosition);
    vctDoubleVec mJointPosition;
    vctDoubleMat mJacobian;
    vctFrm4x4 mCartesianPosition;
    prmPositionCartesianGet mMeasuredCartesianPosition;// for ros publication

    int mNumDof;
    int mNumJoints;

    // constraint controller
    mtsVFController *mController;
    prmKinematicsState mMeasuredKinematics; // follow crtk convention
    prmSensorState mGoalForceValues;
    prmStateJoint mMeasuredJoint;
    mtsVFDataSensorCompliance mCoopObjective; // No additional data needed, therefore using mtsVFBase
    mtsVFDataPlane mPlaneConstraint;
    mtsVFDataJointLimits mJointLimitsConstraint;

    // update optimizer of the information needed
    void updateOptimizerKinematics();
    void updateOptimizerSensor();

    // teleop command
    void servoCartesianForce(const mtsDoubleVec & newGoal);

public:
    // provide a name for the task and define the frequency (time
    // interval between calls to the periodic Run).  Also used to
    // populate the interface(s)
    simpleRobot(const std::string & componentName, const double periodInSeconds);
    ~simpleRobot() {}

    // all four methods are pure virtual in mtsTask
    void Run();        // performed periodically
};

#endif // _simpleRobot_h
