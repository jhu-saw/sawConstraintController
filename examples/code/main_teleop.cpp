//
// Created by max on 2019-10-10.
//

#include "simple_teleop.h"
#include <cstdio>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisstOSAbstraction/osaSleep.h>


int main(int argc, char **argv){
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // component manager
    mtsComponentManager * componentManger = mtsComponentManager::GetInstance();

    // create robot
    simpleTeleop robot("SimpleRobot",200*cmn_ms);

    // add robot to manager
    componentManger->AddComponent(&robot);

    // add ros bridge
    ros::init(argc, argv, "cisst_ros_bridge_example", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;

    mtsROSBridge * subscribers = new mtsROSBridge("subscribers", 0.1 * cmn_ms, &rosNodeHandle);
    subscribers->PerformsSpin(true);
    subscribers->AddSubscriberToCommandWrite<vctFrm4x4, geometry_msgs::PoseStamped>("RequiresSimpleRobot",
                                             "ServoCartesianPosition",
                                             "/simple_robot/servo_cp");
    componentManger->AddComponent(subscribers);

    mtsROSBridge * publishers = new mtsROSBridge("publishers", 5 * cmn_ms, &rosNodeHandle);
    publishers->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>("RequiresSimpleRobot",
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
    ros::spin();

    // cleanup
    componentManger->KillAll();
    componentManger->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManger->Cleanup();

    return 0;
}
