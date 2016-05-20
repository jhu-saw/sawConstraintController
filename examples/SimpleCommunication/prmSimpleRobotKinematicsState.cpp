#include "prmSimpleRobotKinematicsState.h"

CMN_IMPLEMENT_SERVICES(prmSimpleRobotKinematicsState)

//! Updates the kinematics information using the pointer to a joint state. 
/*! Update
*/
void prmSimpleRobotKinematicsState::Update()
{
    //Fill in with robot-specific implementation    
    Jacobian.SetSize(2,2);
    Jacobian.SetAll(0.0);
    Jacobian[0][0] = 1;
    Jacobian[1][1] = 1;    
}
