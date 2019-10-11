//
// Created by max on 2019-10-10.
//

#include "simpleRobot.h"
#include <cisstMultiTask/mtsComponent.h>

int main(int argc, char ** argv){
    // component manager
    mtsComponentManager * componentManger = mtsComponentManager::GetInstance();

    // create robot
    simpleRobot robot("SimpleRobot",200*cmn_ms);

    // add robot to manager
    componentManger->AddComponent(&robot);

    // create components
    componentManger->CreateAll();
    componentManger->WaitForStateAll(mtsComponentState::READY, 2.0*cmn_s);

    // start perodic run
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