/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening
  Created on: 2014

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include "mtsRobotTask.h"
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES(mtsRobotTask);

// Initialize kinematics and VFs
void mtsRobotTask::Setup()
{    
    // Initialize controller with our number of joints (variables to use in the optimizer) and desired form of the output
    Controller = mtsVFController(6, mtsVFBase::JPOS);

    // Kinematics name is important, it is what's used to link VF required kinematics with the appropriate object
    CurrentKinematics.Name = "Current Kinematics";
    DesiredKinematics.Name = "Desired Kinematics";
    // Kinematics can have a pointer to a joint state if needed for kinematics calculations (not used in this example)    
    CurrentKinematics.JointState = &CurrentJointState;    

    CurrentKinematics.Jacobian.SetSize(6, 7, VCT_COL_MAJOR);
    CurrentKinematics.Jacobian.SetAll(0.0);        

    // We have three VFs total, only one is initialized here
    // The way I have it set up, calling update once sets everything up so you never have to call it again if the parameters don't change
    // Here I am not going to change my velocity limits, so I call update in this function which is only called once
    vctDoubleVec UpperVelLimits(6);
    vctDoubleVec LowerVelLimits(6);      
    UpperVelLimits.SetAll(0.1);
    LowerVelLimits.SetAll(-0.1);      
    
    // Each VF has its own requirements for data
    // Joint velocity limits requires a name, and upper/lower limits
    // Fairly simple example, but others may need a kinematics name to get the current jacobian, etc
    Controller.UpdateJointVelLimitsVF("Joint Limits", UpperVelLimits, LowerVelLimits);      
}

void mtsRobotTask::Run()
{
    //Receive messages from other processes (not used in this example, see simple communication example)
    ProcessQueuedEvents();
    ProcessQueuedCommands();    

    // Update Kinematics - The data for this example taken from a snapshot of the UR5 robot state

    // Joint state object that may be used for joint constraints or fed into a kinematics object to construct frame, jacobian, etc
    // That logic not used in this example, numbers are just hard-coded in
    CurrentJointState.JointPosition = vctDoubleVec(6);
    CurrentJointState.JointPosition(0) = -1.6007;
    CurrentJointState.JointPosition(1) = -1.7271;
    CurrentJointState.JointPosition(2) = -2.203;
    CurrentJointState.JointPosition(3) = -0.808;
    CurrentJointState.JointPosition(4) = 1.5951;
    CurrentJointState.JointPosition(5) = -0.031;

    // The frame for current kinematics is just the EE of the UR5 at the time of the snapshot    
    CurrentKinematics.Frame.Rotation()(0,0) = -0.974038;
    CurrentKinematics.Frame.Rotation()(0,1) = 0.214762;
    CurrentKinematics.Frame.Rotation()(0,2) = 0.071604;
    CurrentKinematics.Frame.Rotation()(1,0) = 0.218701;
    CurrentKinematics.Frame.Rotation()(1,1) = 0.974375;
    CurrentKinematics.Frame.Rotation()(1,2) = 0.0525682;
    CurrentKinematics.Frame.Rotation()(2,0) = 0.0584795;
    CurrentKinematics.Frame.Rotation()(2,1) = 0.0668633;
    CurrentKinematics.Frame.Rotation()(2,2) = -0.996047;
    CurrentKinematics.Frame.Translation() = vct3(-0.113387,-0.415728,-0.176139);
    
    // I make a random incremental frame so we are moving somewhere new each iteration for the desired frame
    double rand_x_axis = -1.0 + (double)rand() / RAND_MAX * 2.0;
    double rand_y_axis = -1.0 + (double)rand() / RAND_MAX * 2.0;
    double rand_z_axis = -1.0 + (double)rand() / RAND_MAX * 2.0;
    vct3 rand_axis(rand_x_axis, rand_y_axis, rand_z_axis);
    rand_axis = rand_axis / rand_axis.Norm();
    double rand_angle = -0.5 + (double)rand() / RAND_MAX * 1.0;
    vctMatRot3 dR(vctAxAnRot3 (rand_axis, rand_angle));
    dR.FromNormalized(dR);
    double rand_x_trans = -0.05 + (double)rand() / RAND_MAX * 0.1;
    double rand_y_trans = -0.05 + (double)rand() / RAND_MAX * 0.1;
    double rand_z_trans = -0.05 + (double)rand() / RAND_MAX * 0.1;
    vct3 dx(rand_x_trans, rand_y_trans, rand_z_trans);
    vctFrm3 dF(dR,dx);
    DesiredKinematics.Frame = CurrentKinematics.Frame * dF;   

    // Adds this newly update kinematics object to the controller (so VFs can look it up by name)
    Controller.SetKinematics(DesiredKinematics);  
 
    // UR5 Jacobian captured from snapshot
    Jacobian = vctDoubleMat(6,6);
    Jacobian(0,0) = 0.415728;
    Jacobian(0,1) = -0.00794507;
    Jacobian(0,2) = -0.0204973;
    Jacobian(0,3) = -0.0121787;
    Jacobian(0,4) = 0.405041;
    Jacobian(0,5) = 0.00797124;
    Jacobian(1,0) = -0.113387;
    Jacobian(1,1) = -0.26561;
    Jacobian(1,2) = -0.685241;
    Jacobian(1,3) = -0.407142;
    Jacobian(1,4) = -0.0123278;
    Jacobian(1,5) = 0.00136325;
    Jacobian(2,0) = 0.0;
    Jacobian(2,1) = 0.418932;
    Jacobian(2,2) = 0.352773;
    Jacobian(2,3) = 0.0762744;
    Jacobian(2,4) = 0.00824082;
    Jacobian(2,5) = 0.000233927;
    Jacobian(3,0) = 0.0;
    Jacobian(3,1) = -0.999553;
    Jacobian(3,2) = -0.999553;
    Jacobian(3,3) = -0.999553;
    Jacobian(3,4) = -0.0298893;
    Jacobian(3,5) = 0.0250588;
    Jacobian(4,0) = 0.0;
    Jacobian(4,1) = 0.0298992;
    Jacobian(4,2) = 0.0298992;
    Jacobian(4,3) = 0.0298992;
    Jacobian(4,4) = -0.999223;
    Jacobian(4,5) = 0.0249625;
    Jacobian(5,0) = 1.0;
    Jacobian(5,1) = 0.0;
    Jacobian(5,2) = 0.0;
    Jacobian(5,3) = 0.0;
    Jacobian(5,4) = -0.0257082;
    Jacobian(5,5) = -0.999374;
    CurrentKinematics.Jacobian = Jacobian;

    // Adds this new data for the current kinematics to the controller
    Controller.SetKinematics(CurrentKinematics);
        
    // Update objective function, this does the following:
    // Jac = CurrentKinematics.Jacobian
    // dx = DesiredKinematics.Frame.Translation() - CurrentKinematics.Frame.Translation()
    // These kinematics objects are looked up in the kinematics list by the names provided
    // The false here means that we don't care about aligning the rotational poses of the two frames, just translation
    Controller.UpdateFollowPathVF("Follow Objective", "Current Kinematics", "Desired Kinematics", false);

    // Update RCM parameters
    vct3 RCMPoint(-0.389107,-0.340498,0.13305);
    vctDoubleMat JacClosest(6,6);
    JacClosest(0,0) = 0.431763;
    JacClosest(0,1) = 0.00170167;
    JacClosest(0,2) = -0.0108506;
    JacClosest(0,3) = -0.00253192;
    JacClosest(0,4) = 0.0822374;
    JacClosest(0,5) = 0.0;
    JacClosest(1,0) = -0.120113;
    JacClosest(1,1) = 0.0568881;
    JacClosest(1,2) = -0.362743;
    JacClosest(1,3) = -0.0846439;
    JacClosest(1,4) = -0.00251137;
    JacClosest(1,5) = 0.0;
    JacClosest(2,0) = 0.0;
    JacClosest(2,1) = 0.435161;
    JacClosest(2,2) = 0.369002;
    JacClosest(2,3) = 0.0925036;
    JacClosest(2,4) = 0.00199933;
    JacClosest(2,5) = 0.0;
    JacClosest(3,0) = 0.0;
    JacClosest(3,1) = -0.999553;
    JacClosest(3,2) = -0.999553;
    JacClosest(3,3) = -0.999553;
    JacClosest(3,4) = -0.0298893;
    JacClosest(3,5) = 0.0250588;
    JacClosest(4,0) = 0.0;
    JacClosest(4,1) = 0.0298992;
    JacClosest(4,2) = 0.0298992;
    JacClosest(4,3) = 0.0298992;
    JacClosest(4,4) = -0.999223;
    JacClosest(4,5) = 0.0249625;
    JacClosest(5,0) = 1.0;
    JacClosest(5,1) = 0.0;
    JacClosest(5,2) = 0.0;
    JacClosest(5,3) = 0.0;
    JacClosest(5,4) = -0.0257082;
    JacClosest(5,5) = -0.999374;
    vctFrm3 TipFrame;
    TipFrame.Rotation()(0,0) = -0.974038;
    TipFrame.Rotation()(0,1) = 0.214762;
    TipFrame.Rotation()(0,2) = 0.071604;
    TipFrame.Rotation()(1,0) = 0.218701;
    TipFrame.Rotation()(1,1) = 0.974375;
    TipFrame.Rotation()(1,2) = 0.0525682;
    TipFrame.Rotation()(2,0) = -0.0584795;
    TipFrame.Rotation()(2,1) = 0.0668633;
    TipFrame.Rotation()(2,2) = -0.996047;       
    TipFrame.Translation() = vct3(-0.113387,-0.415728,-0.176139);         
    // Again, we are updating a VF based on the latest info
    // We need to recalculate the closest point on the axis to the rcm point and jacobian of that point and pass that and the end effector frame into the controller       
    Controller.UpdateRCMVF(3,"RCM","Current Kinematics",RCMPoint,JacClosest,TipFrame);  
    

    // deactivate all VFs, activate follow only
    Controller.DeactivateAll();
    Controller.ActivateVF("Follow Objective");

    // solve and check output
    // output should bring you to desired position    

    // Go through the VF list, update state data pointers, assign tableau references, and fill in the references
    Controller.UpdateOptimizer(this->GetPeriodicity());

    // Compute a new motion and check the solve return code
    OptimizerStatus = Controller.Solve(ControllerOutput);

    std::cout << "Follow: " << ControllerOutput << std::endl;

    // deactivate all VFs, activate both follow and joint lims    
    Controller.DeactivateAll();
    
    Controller.ActivateVF("Follow Objective");
    
    Controller.ActivateVF("Joint Limits");

    // solve and check output
    // output should bring you closer to desired position, but shouldn't exceed limits
    
    // Go through the VF list, update state data pointers, assign tableau references, and fill in the references
    Controller.UpdateOptimizer(this->GetPeriodicity());

    // Compute a new motion and check the solve return code
    OptimizerStatus = Controller.Solve(ControllerOutput);

    std::cout << "Follow and Joint Limits: " << ControllerOutput << std::endl;

    // deactivate all VFs, activate both follow and rcm
    Controller.DeactivateAll();
    Controller.ActivateVF("Follow Objective");
    Controller.ActivateVF("RCM");

    // solve and check output
    // output should bring you to desired position and RCM shouldn't be violated
    
    // Go through the VF list, update state data pointers, assign tableau references, and fill in the references
    Controller.UpdateOptimizer(this->GetPeriodicity());

    // Compute a new motion and check the solve return code
    OptimizerStatus = Controller.Solve(ControllerOutput);

    std::cout << "Follow and RCM: " << ControllerOutput << std::endl;


}
