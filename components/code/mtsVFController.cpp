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

void mtsVFController::UpdateFollowPathVF(const std::string & vfName,
                                                          const std::string & CurKinName,
                                                          const std::string & DesKinName,
                                                          const bool & UseRotation)
{    
    FollowData.Name = vfName;
    if(!UseRotation)
    {
        FollowData.ObjectiveRows = 3;
    }
    else
    {
        FollowData.ObjectiveRows = 6;
    }
    FollowData.KinNames.clear();
    FollowData.KinNames.push_back(CurKinName);
    FollowData.KinNames.push_back(DesKinName);
    AddVFFollowPath(FollowData);
}

void mtsVFController::UpdateJointVelLimitsVF(const std::string vfName, const vctDoubleVec & UpperLimits, const vctDoubleVec & LowerLimits)
{    
    // todo assert sizes of upper and lower limits    
    JLimitsData.UpperLimits = UpperLimits;
    JLimitsData.LowerLimits = LowerLimits;
    JLimitsData.Name = vfName;
    JLimitsData.IneqConstraintRows = UpperLimits.size() + LowerLimits.size();
    JLimitsData.KinNames.clear();
    AddVFJointLimits(JLimitsData);
}

void mtsVFController::UpdateJointPosLimitsVF(const std::string vfName, const vctDoubleVec & UpperLimits, const vctDoubleVec & LowerLimits, const vctDoubleVec & CurrentJoints)
{    
    // todo assert sizes of upper and lower limits
    AJLimitsData.UpperLimits = UpperLimits;
    AJLimitsData.LowerLimits = LowerLimits;
    AJLimitsData.Name = vfName;
    AJLimitsData.IneqConstraintRows = UpperLimits.size() + LowerLimits.size();
    AJLimitsData.KinNames.clear();
    AJLimitsData.CurrentJoints.SetSize(UpperLimits.size());
    AJLimitsData.CurrentJoints = CurrentJoints;
    AddVFAbsoluteJointLimits(AJLimitsData);
}

void mtsVFController::UpdatePlaneVF(const std::string vfName, const std::string curKinName)
{    
    PlaneData.IneqConstraintRows = 1;
    PlaneData.Name = vfName;
    PlaneData.KinNames.clear();
    PlaneData.KinNames.push_back(curKinName);
    AddVFPlane(PlaneData);
}

void mtsVFController::UpdateRCMVF(const size_t rows, const std::string vfName, const std::string curKinName, const vct3 & RCMPoint, const vctDoubleMat & JacClosest, const vctFrm3 & TipFrame)
{    
    RCM_Data.IneqConstraintRows = rows;    
    RCM_Data.Name = vfName;
    RCM_Data.KinNames.clear();      
    RCM_Data.KinNames.push_back(curKinName);    
    RCM_Data.JacClosest = JacClosest;    
    RCM_Data.TipFrame = TipFrame;
    AddVFRCM(RCM_Data);
}

bool mtsVFController::SetVFData(const mtsVFDataBase & data, const std::type_info & type)
{
    // find vf by data.Name
    std::map<std::string, mtsVFBase *>::iterator itVF;
    itVF = VFMap.find(data.Name);

    // if not found, return false
    if(itVF == VFMap.end())
    {
        return false;
    }

    // if found, get std::typeid(iter->second) and compare to type
    // if same type, just update iter->second->Data and return true
    if(typeid(itVF->second) == type)
    {
        DecrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        itVF->second->Data = new mtsVFDataBase(data);
        IncrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        return true;
    }
    return false;
}

bool mtsVFController::SetVFDataSensorCompliance(const mtsVFDataSensorCompliance & data, const std::type_info & type)
{
    // find vf by data.Name
    std::map<std::string, mtsVFBase *>::iterator itVF;
    itVF = VFMap.find(data.Name);

    // if not found, return false
    if(itVF == VFMap.end())
    {
        return false;
    }

    // if found, get std::typeid(iter->second) and compare to type
    // if same type, just update iter->second->Data and return true
    if(typeid(itVF->second) == type)
    {
        DecrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        itVF->second->Data = new mtsVFDataSensorCompliance(data);
        IncrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        return true;
    }
    return false;
}

bool mtsVFController::SetVFDataPlane(const mtsVFDataPlane &data, const std::type_info &type)
{
    // find vf by data.Name
    std::map<std::string, mtsVFBase *>::iterator itVF;
    itVF = VFMap.find(data.Name);

    // if not found, return false
    if(itVF == VFMap.end())
    {
        return false;
    }

    // if found, get std::typeid(iter->second) and compare to type
    // if same type, just update iter->second->Data and return true
    if(typeid(itVF->second) == type)
    {
        DecrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        itVF->second->Data = new mtsVFDataPlane(data);
        IncrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        return true;
    }
    return false;
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
        IncrementUsers(vf.KinNames,vf.SensorNames);
    // }
}

//! Adds/updates a basic virtual fixture that uses incremental joint position control in the map and increments users of kinematics and sensors
/*! SetVFJointPos
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFJointPosition(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    if (!SetVFData(vf, typeid(mtsVFJointPosition)))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFBase *>(vf.Name,new mtsVFJointPosition(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
        IncrementUsers(vf.KinNames,vf.SensorNames);
    }
}

//! Adds/updates a basic virtual fixture that uses cartesian velocity control in the map and increments users of kinematics and sensors
/*! SetVFCartVel
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFCartesianTranslation(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    if (!SetVFData(vf, typeid(mtsVFCartesianTranslation)))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFBase *>(vf.Name,new mtsVFCartesianTranslation(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
        IncrementUsers(vf.KinNames,vf.SensorNames);
    }
}


//! Adds/updates a basic virtual fixture that uses cartesian incremental position control in the map and increments users of kinematics and sensors
/*! SetVFCartPos
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFCartesianOrientation(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    if (!SetVFData(vf, typeid(mtsVFCartesianOrientation)))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFBase *>(vf.Name,new mtsVFCartesianOrientation(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
        IncrementUsers(vf.KinNames,vf.SensorNames);
    }
}

//! Adds/updates a sensor compliance virtual fixture in the map and increments users of kinematics and sensors
/*! SetVFSensorCompliance
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFSensorCompliance(const mtsVFDataSensorCompliance & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    if (!SetVFDataSensorCompliance(vf, typeid(mtsVFSensorCompliance)))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFSensorCompliance *>(vf.Name,new mtsVFSensorCompliance(vf.Name,new mtsVFDataSensorCompliance(vf))));
        // Increment users of each kinematics and sensor object found
        IncrementUsers(vf.KinNames,vf.SensorNames);
    }
}

//! Adds/updates a sensor compliance virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFPlane
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFPlane(const mtsVFDataPlane & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    if (!SetVFDataPlane(vf, typeid(mtsVFPlane)))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFPlane *>(vf.Name, new mtsVFPlane(vf.Name,new mtsVFDataPlane(vf))));
        // Increment users of each kinematics and sensor object found
        IncrementUsers(vf.KinNames,vf.SensorNames);
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
       IncrementUsers(vf.KinNames,vf.SensorNames);
   // }
}

//! Adds/updates a path-following virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFFollowPath
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFFollowPath(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
   // if (!SetVFData(vf, typeid(mtsVFFollow)))
   // {
       // Adds a new virtual fixture to the active vector
       VFMap[vf.Name] = new mtsVFFollow(vf.Name,new mtsVFDataBase(vf));
       // std::cout << "Added new VF: " << typeid(VFMap.find(vf.Name)->second).name() << std::endl;
       // Increment users of each kinematics and sensor object found
       IncrementUsers(vf.KinNames,vf.SensorNames);
   // }
}

//! Adds/updates a velocity-limiting virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFJointLimits
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFJointLimits(const mtsVFDataJointLimits & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
   if (!SetVFData(vf, typeid(mtsVFJointLimits)))
   {
       // Adds a new virtual fixture to the active vector
       VFMap.insert(std::pair<std::string,mtsVFJointLimits *>(vf.Name,new mtsVFJointLimits(vf.Name,new mtsVFDataJointLimits(vf))));
       // Increment users of each kinematics and sensor object found
       IncrementUsers(vf.KinNames,vf.SensorNames);
   }
}

//! Adds/updates a velocity-limiting virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFAbsoluteJointLimits
@param vf virtual fixture to be added
*/
void mtsVFController::AddVFAbsoluteJointLimits(const mtsVFDataAbsoluteJointLimits & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
   if (!SetVFDataAJL(vf, typeid(mtsVFAbsoluteJointLimits)))
   {
       // Adds a new virtual fixture to the active vector
       VFMap[vf.Name] = new mtsVFAbsoluteJointLimits(vf.Name,new mtsVFDataAbsoluteJointLimits(vf));
       // Increment users of each kinematics and sensor object found
//       IncrementUsers(vf.KinNames,vf.SensorNames);
   }
}

//TODO either change this to match other SETVF methods or change other methods to match this one
bool mtsVFController::SetVFDataRCM(const mtsVFDataRCM & data, const std::type_info & type)
{
    // find vf by data.Name
    std::map<std::string, mtsVFBase *>::iterator itVF;
    itVF = VFMap.find(data.Name);

    // if not found, return false
    if(itVF == VFMap.end())
    {     
        return false;
    }    

    // if found, get std::typeid(iter->second) and compare to type
    // if same type, just update iter->second->Data and return true
    if(typeid(itVF->second) == type)
    {     
        DecrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        itVF->second->Data = new mtsVFDataRCM(data);
        IncrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        return true;
    }    
    return false;
}

//TODO either change this to match other SETVF methods or change other methods to match this one
bool mtsVFController::SetVFDataAJL(const mtsVFDataAbsoluteJointLimits & data, const std::type_info & type)
{
    // find vf by data.Name
    std::map<std::string, mtsVFBase *>::iterator itVF;
    itVF = VFMap.find(data.Name);

    // if not found, return false
    if(itVF == VFMap.end())
    {
        return false;
    }

    // if found, get std::typeid(iter->second) and compare to type
    // if same type, just update iter->second->Data and return true
    if(typeid(itVF->second) == type)
    {        
        DecrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        itVF->second->Data = new mtsVFDataAbsoluteJointLimits(data);
        IncrementUsers(itVF->second->Data->KinNames,itVF->second->Data->SensorNames);
        return true;
    }    
    return false;
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
    else
    {
        itVF->second->Data->Active = true;
    }
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
        if(itVF->second->Data->Active)
        {
            itVF->second->ReserveSpace(Optimizer);
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
            
            //updates the virtual fixture's kinematics and sensor objects
            tempVFData->LookupStateData(Kinematics,Sensors);

            //updates the virtual fixture's references to the control optimizer tableau
            tempVFData->SetTableauRefs(Optimizer);

            //fills in the tableau (subclasses of mtsVFData override this method with their own logic)
            tempVFData->FillInTableauRefs(ControllerMode,TickTime);

        }
    }
}

//! Solves the constraint optimization problem and fills the result into the parameter
/*! Solve
  @param dq Storage for the result of the solve call
  */
nmrConstraintOptimizer::STATUS mtsVFController::Solve(vctDoubleVec & dq)
{
    return Optimizer.Solve(dq);
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

//! Helper function for incrementing the users of sensors and kinematics that a new VF requires
/*! IncrementUsers
  */
void mtsVFController::IncrementUsers(const std::vector<std::string> kin_names, const std::vector<std::string> sensor_names)
{
    std::map<std::string,prmKinematicsState *>::iterator itKin;
    std::map<std::string,prmSensorState *>::iterator itSen;

    //increment kinematics users
    for(size_t i = 0; i < kin_names.size(); i++)
    {
        itKin = Kinematics.find(kin_names.at(i));
        if(itKin != Kinematics.end())
        {
            itKin->second->UserCount++;
        }
    }

    //increment sensor users
    for(size_t i = 0; i < sensor_names.size(); i++)
    {
        itSen = Sensors.find(sensor_names.at(i));
        if(itSen != Sensors.end())
        {
            itSen->second->UserCount++;
        }
    }
}

void mtsVFController::DecrementUsers(const std::vector<std::string> kin_names, const std::vector<std::string> sensor_names)
{
    // Reduce the user counts for any kinematics objects used by this VF
    std::map<std::string,prmKinematicsState *>::iterator itKin;
    for(size_t i = 0; i < kin_names.size(); i++)
    {
        itKin = Kinematics.find(kin_names.at(i));
        if(itKin != Kinematics.end())
        {
            itKin->second->UserCount--;
        }
    }

    // Similarly, reduce the user counts for sensor objects used by this VF
    std::map<std::string,prmSensorState *>::iterator itSen;
    for(size_t i = 0; i < sensor_names.size(); i++)
    {
        itSen = Sensors.find(sensor_names.at(i));
        if(itSen != Sensors.end())
        {
            itSen->second->UserCount--;
        }
    }
}