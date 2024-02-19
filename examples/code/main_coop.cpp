//
// Created by max on 2019-10-10.
//

#include "simple_coop.h"
#include <cstdio>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisstOSAbstraction/osaSleep.h>


int main(int argc, char ** argv){
    // component manager
    mtsComponentManager * componentManger = mtsComponentManager::GetInstance();

    // create robot
    simpleCoop robot("SimpleRobot",200*cmn_ms);

    // add robot to manager
    componentManger->AddComponent(&robot);

    // add ros bridge
    cisst_ral::ral ral(argc, argv, "cisst_ros_bridge_example");
    auto rosNode = ral.node();

    mtsROSBridge * subscribers = new mtsROSBridge("subscribers", 0.1 * cmn_ms, rosNode);
    subscribers->PerformsSpin(true);
    subscribers->AddSubscriberToCommandWrite<mtsDoubleVec,
                                             CISST_RAL_MSG(geometry_msgs, Wrench)>("RequiresSimpleRobot",
                                                                                   "ServoCartesianForce",
                                                                                   "/simple_robot/servo_cf");
    componentManger->AddComponent(subscribers);

    mtsROSBridge * publishers = new mtsROSBridge("publishers", 5 * cmn_ms, rosNode);
    publishers->AddPublisherFromCommandRead<prmPositionCartesianGet,
                                            CISST_RAL_MSG(geometry_msgs, PoseStamped)>("RequiresSimpleRobot",
                                                                                       "GetMeasuredCartesianPosition",
                                                                                       "/simple_robot/measured_cp");
    componentManger->AddComponent(publishers);

    // connect components
    componentManger->Connect(subscribers->GetName(), "RequiresSimpleRobot",
                             robot.GetName(), "ProvidesSimpleRobot");
    componentManger->Connect(publishers->GetName(), "RequiresSimpleRobot",
                             robot.GetName(), "ProvidesSimpleRobot");

    // create components
    componentManger->CreateAll();
    componentManger->WaitForStateAll(mtsComponentState::READY, 2.0*cmn_s);

    // start periodic run
    componentManger->StartAll();
    componentManger->WaitForStateAll(mtsComponentState::ACTIVE, 2.0*cmn_s);

    // ros::spin() callback for subscribers
    cisst_ral::spin(rosNode);

    // cleanup
    componentManger->KillAll();
    componentManger->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManger->Cleanup();

    return 0;
}
