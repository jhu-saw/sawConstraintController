//
// Created by max on 2019-10-10.
//

#include "simpleRobot.h"
#include <cstdio>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisstMultiTask/mtsComponent.h>

int main(int argc, char ** argv){
    // component manager
    mtsComponentManager * componentManger = mtsComponentManager::GetInstance();

    // create robot
    simpleRobot robot("SimpleRobot",200*cmn_ms);

    // add robot to manager
    componentManger->AddComponent(&robot);
    // add ros bridge
    mtsROSBridge * subscribers = new mtsROSBridge("subscribers", 0.1 * cmn_ms, true /* spin */);
    subscribers->AddSubscriberToCommandWrite<mtsFrm4x4, geometry_msgs::TransformStamped>("RequiresSimpleRobot",
                                             "ServoCartesianRelative",
                                             "/simple_robot/servo_cr");
    componentManger->AddComponent(subscribers);

    // connect components
    componentManger->Connect(subscribers->GetName(), "RequiresSimpleRobot",
            robot.GetName(), "ProvidesSimpleRobot");

    // create components
    componentManger->CreateAll();
    componentManger->WaitForStateAll(mtsComponentState::READY, 2.0*cmn_s);

    // start periodic run
    componentManger->StartAll();
    componentManger->WaitForStateAll(mtsComponentState::ACTIVE, 2.0*cmn_s);

    while(true){
    }

    // cleanup
    componentManger->KillAll();
    componentManger->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManger->Cleanup();

    return 0;
}