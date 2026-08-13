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

#include <sawConstraintController/mtsVFController.h>

CMN_IMPLEMENT_SERVICES(mtsVFController)


bool mtsVFController::SetVFData(const mtsVFDataBase & data)
{
    // find vf by data.Name
    std::map<std::string, mtsVFBase *>::iterator itVF;
    itVF = VFMap.find(data.Name);

    // if not found, return false
    if(itVF == VFMap.end())
    {
        return false;
    }

    return true;
}

//! Adds/updates a basic virtual fixture that uses joint velocity control in the map and increments users of kinematics and sensors
/*! SetVFJointVel
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFJointVelocity(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    // if (!SetVFData(vf, typeid(mtsVFJointVelocity)))
    // {
        // Add a new virtual fixture to the active vector
        VFMap[vf.Name] = new mtsVFJointVelocity(vf.Name,new mtsVFDataBase(vf));
        // Increment users of each kinematics and sensor object found
//        IncrementUsers(vf.KinNames,vf.SensorNames);
    // }
}

//! Adds/updates a basic virtual fixture that uses incremental joint position control in the map and increments users of kinematics and sensors
/*! SetVFJointPos
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFJointPosition(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    // if (!SetVFData(vf, typeid(mtsVFJointPosition)))
    // {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFBase *>(vf.Name,new mtsVFJointPosition(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
//        IncrementUsers(vf.KinNames,vf.SensorNames);
    // }
}

//! Adds/updates a basic virtual fixture that uses cartesian velocity control in the map and increments users of kinematics and sensors
/*! SetVFCartVel
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFCartesianTranslation(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    // if (!SetVFData(vf, typeid(mtsVFCartesianTranslation)))
    // {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFBase *>(vf.Name,new mtsVFCartesianTranslation(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
//        IncrementUsers(vf.KinNames,vf.SensorNames);
    // }
}


//! Adds/updates a basic virtual fixture that uses cartesian incremental position control in the map and increments users of kinematics and sensors
/*! SetVFCartPos
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFCartesianOrientation(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    // if (!SetVFData(vf, typeid(mtsVFCartesianOrientation)))
    // {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFBase *>(vf.Name,new mtsVFCartesianOrientation(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
//        IncrementUsers(vf.KinNames,vf.SensorNames);
    // }
}

//! Adds/updates a sensor compliance virtual fixture in the map and increments users of kinematics and sensors
/*! SetVFSensorCompliance
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFSensorCompliance(mtsVFDataSensorCompliance & vf)
{
    if (!SetVFData(vf))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFSensorCompliance *>(vf.Name,new mtsVFSensorCompliance(vf.Name,&vf)));
    }
}

//! Adds/updates a sensor compliance virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFPlane
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFPlane(mtsVFDataPlane &vf)
{
    if (!SetVFData(vf))
    {
        VFMap.insert(std::pair<std::string, mtsVFPlane*>(vf.Name, new mtsVFPlane(vf.Name, &vf)));
    }
}

//! Adds/updates an RCM virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFRCM
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFRCM(const mtsVFDataRCM & vf)
{    
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
   // if (!SetVFDataRCM(vf, typeid(mtsVF_RCM)))
   // {
       // Adds a new virtual fixture to the active vector
       // std::cout << "Adding new VF" << std::endl;       
    
       // VFMap.insert(std::pair<std::string,mtsVF_RCM *>(vf.Name,new mtsVF_RCM(vf.Name,new mtsVFDataRCM(vf))));       
       VFMap[vf.Name] = new mtsVF_RCM(vf.Name,new mtsVFDataRCM(vf));

       // std::cout << "Added new VF: " << typeid(VFMap.find(vf.Name)->second).name() << std::endl;
       // Increment users of each kinematics and sensor object found
//       IncrementUsers(vf.KinNames,vf.SensorNames);
   // }
}

//! Adds/updates a path-following virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFFollowPath
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFFollow(mtsVFDataBase & vf)
{
    if (!SetVFData(vf))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFFollow *>(vf.Name,new mtsVFFollow(vf.Name,&vf)));
    }
}

void mtsVFController::AddVFCylinder(mtsVFDataCylinder &vf)
{
    if (!SetVFData(vf))
    {
        VFMap.insert(std::pair<std::string, mtsVFCylinder*>(vf.Name, new mtsVFCylinder(vf.Name, new mtsVFDataCylinder(vf))));
    }
}

//! Adds/updates a velocity-limiting virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFJointLimits
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFLimits(mtsVFDataJointLimits &vf)
{
    if (!SetVFData(vf))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFLimitsConstraint *>(vf.Name,new mtsVFLimitsConstraint(vf.Name,&vf)));
    }
}

//! Adds/updates a velocity-limiting virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFCartesianLimits
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFCartesianLimits(const mtsVFDataJointLimits & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
   // if (!SetVFData(vf, typeid(mtsVFJointLimits)))
   // {
       // Adds a new virtual fixture to the active vector
       VFMap.insert(std::pair<std::string,mtsVFCartesianLimits *>(vf.Name,new mtsVFCartesianLimits(vf.Name,new mtsVFDataJointLimits(vf))));
       // Increment users of each kinematics and sensor object found
//       IncrementUsers(vf.KinNames,vf.SensorNames);
   // }
}

bool mtsVFController::ActivateVF(const std::string & s)
{
    // find vf by data.Name
    std::map<std::string, mtsVFBase *>::iterator itVF;
    itVF = VFMap.find(s);

    // if not found, return false
    if(itVF == VFMap.end())
    {
        return false;
    }
    itVF->second->Data->Active = true;
    return true;
}

void mtsVFController::DeactivateAll()
{
    std::map<std::string,mtsVFBase *>::iterator itVF;
    for(itVF = VFMap.begin(); itVF != VFMap.end(); itVF++)
    {
        if(itVF->second->Data->Active)
        {
            itVF->second->Data->Active = false;
        }
    }
}

//! Adds/updates a kinematics in the map
/*! SetKinematics
@param kin kinematics object to be added
*/
void mtsVFController::SetKinematics(const prmKinematicsState & kin)
{
    RemoveKinematicsFromMap(kin.Name);
    Kinematics.insert(std::pair<std::string, prmKinematicsState *>(kin.Name,new prmKinematicsState(kin)));
}

//! Removes a kinematics object from the map
/*! RemoveKinematicsFromMap
@param kinName name of kinematics object to be removed
*/
void mtsVFController::RemoveKinematicsFromMap(const std::string & kinName)
{
    // Removes a kinematics object from the map
    std::map<std::string, prmKinematicsState *>::iterator itKin;
    itKin = Kinematics.find(kinName);
    if(itKin == Kinematics.end())
    {
        return;  // not found, so no action required
    }

    prmKinematicsState * kinP = itKin->second;

    // Now remove from the map and delete the old object
    Kinematics.erase(itKin);
    delete kinP;
}

//! Adds/updates a sensor in the map
/*! SetSensor
@param sen sensor state to be added
*/
void mtsVFController::SetSensor(const prmSensorState & sen)
{
    RemoveSensorFromMap(sen.Name);
    Sensors.insert(std::pair<std::string, prmSensorState *>(sen.Name,new prmSensorState(sen)));
}

//! Adds/Updates a sensor to the map
void mtsVFController::SetSensorOffset(const prmOffsetState & sen)
{
    RemoveSensorFromMap(sen.Name);
    Sensors.insert(std::pair<std::string, prmSensorState *>(sen.Name,new prmOffsetState(sen)));
}

//! Removes a sensor from the map
/*! RemoveSensorFromMap
@param senName name of sensor state to be removed
*/
void mtsVFController::RemoveSensorFromMap(const std::string & senName)
{
    // Removes a sensor from the map
    std::map<std::string, prmSensorState *>::iterator itSen;
    itSen = Sensors.find(senName);
    if(itSen == Sensors.end())
    {
        return;  // not found, so no action required
    }

    prmSensorState * senP = itSen->second;

    // Now remove from the map and delete the old object
    Sensors.erase(itSen);
    delete senP;
}

//! Reallocates the tableau, assigns references to it for the virtual fixtures, and instructs the virtual fixtures to fill their references in
/*! UpdateCO
*/
void mtsVFController::UpdateOptimizer(double TickTime)
{
    // use VFVector to find the space needed in the control optimizer tableau
    Optimizer.ResetIndices();

    std::map<std::string,mtsVFBase *>::iterator itVF;
    for(itVF = VFMap.begin(); itVF != VFMap.end(); itVF++)
    {       
        mtsVFBase * tempVFData = itVF->second;
        if(tempVFData->Data->Active)
        {
            //updates the virtual fixture's kinematics and sensor objects
            tempVFData->LookupStateData(Kinematics,Sensors);

            tempVFData->ReserveSpace(Optimizer);
        }
    }


    //allocate the control optimizer matrices and vectors according to its indices
    Optimizer.Allocate();

    // go through virtual fixtures again and fill in tableau
    Optimizer.ResetIndices();    

    for(itVF = VFMap.begin(); itVF != VFMap.end(); itVF++)
    {
        mtsVFBase * tempVFData = itVF->second;        

        if(tempVFData->Data->Active)
        {
            //updates the virtual fixture's references to the control optimizer tableau
            tempVFData->SetTableauRefs(Optimizer);

            //fills in the tableau (subclasses of mtsVFData override this method with their own logic)
            tempVFData->FillInTableauRefs(ControllerMode,TickTime);

        }
    }
//    CMN_LOG_CLASS_RUN_VERBOSE << "objective m" << std::endl;
//    CMN_LOG_CLASS_RUN_VERBOSE << Optimizer.GetObjectiveMatrix() << std::endl;
//    CMN_LOG_CLASS_RUN_VERBOSE << "objective v" << std::endl;
//    CMN_LOG_CLASS_RUN_VERBOSE << Optimizer.GetObjectiveVector() << std::endl;
//    CMN_LOG_CLASS_RUN_VERBOSE << "ineq m" << std::endl;
//    CMN_LOG_CLASS_RUN_VERBOSE << Optimizer.GetIneqConstraintMatrix() << std::endl;
//    CMN_LOG_CLASS_RUN_VERBOSE << "ineq v" << std::endl;
//    CMN_LOG_CLASS_RUN_VERBOSE << Optimizer.GetIneqConstraintVector() << std::endl;
}

//! Solves the constraint optimization problem and fills the result into the parameter
//! To enhance numerical stability, round to nearest 6 decimal places
//! If unit is in meters, this results in micron meter precision
//! If unit is in mili-meters, this results in pico meter precision
/*! Solve
  @param dq Storage for the result of the solve call
  */
nmrConstraintOptimizer::STATUS mtsVFController::Solve(vctDoubleVec & dq)
{
    nmrConstraintOptimizer::STATUS status = Optimizer.Solve(dq);
    round6(dq);
    return status;
}

void mtsVFController::LookupBaseData()
{
    std::map<std::string, prmKinematicsState *>::iterator kinIt;
    for(kinIt = Kinematics.begin(); kinIt != Kinematics.end(); kinIt++)
    {
        if(kinIt->second->NeedsBase)
        {
            kinIt->second->LookupKinematics(Kinematics);
        }
    }
    std::map<std::string, prmSensorState *>::iterator senIt;
    for(senIt = Sensors.begin(); senIt != Sensors.end(); senIt++)
    {
        if(senIt->second->NeedsBase)
        {
            senIt->second->LookupSensor(Sensors);
        }
    }
}
