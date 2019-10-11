//
// Created by max on 2019-10-11.
//

#ifndef SAWCONSTRAINTCONTROLLER_SIMPLEROBOT_H
#define SAWCONSTRAINTCONTROLLER_SIMPLEROBOT_H

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstVector/vctDynamicVectorTypes.h>

class simpleRobot: public mtsTaskPeriodic {
public:
    simpleRobot(const std::string & componentName, double periodInSeconds);
    ~simpleRobot(){};
    void Run();
private:
    void init();
    vctDoubleMat mJacobian;
    vctFrm4x4 mCartesianPosition;
    int mNumDof;
    int mNumJoints;
};


#endif //SAWCONSTRAINTCONTROLLER_SIMPLEROBOT_H
