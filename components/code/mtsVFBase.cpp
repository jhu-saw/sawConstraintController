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

#include <sawConstraintController/mtsVFBase.h>

CMN_IMPLEMENT_SERVICES(mtsVFBase)

std::string mtsVFBase::DefaultKinematicsName = "";

/*! Constructor
\param name String name of object
*/
mtsVFBase::mtsVFBase(const std::string & name, mtsVFDataBase * data)
{
    Name = name;
    Data = data;
    Kinematics.clear();
    Sensors.clear();    

/*
    @TODO
    If no kinematics specified
*/
//    data->KinNames.push_back(DefaultKinematicsName);
}

//! Reserves space in the control optimizer.
/*! reserve_space
\param co Control optimizer object
*/
void mtsVFBase::ReserveSpace(nmrConstraintOptimizer & co)
{
    ComputeConstraintSize();
    co.ReserveSpace(Data->ObjectiveRows,Data->IneqConstraintRows,Data->EqConstraintRows,Data->NumSlacks);
}

//! Updates internal state data.
/*! LookupStateData
\param k A map of names to kinematics objects
\param s A map of names to sensor objects
*/
void mtsVFBase::LookupStateData(const std::map<std::string,prmKinematicsState *> & k, const std::map<std::string,prmSensorState *> & s)
{
    //update pointers to state data
    Kinematics.clear();
    std::map<std::string,prmKinematicsState *>::const_iterator itKin;
    std::map<std::string,prmSensorState *>::const_iterator itSen;    

    //find kinematics objects by name, update pointers
    for(size_t i = 0; i < Data->KinNames.size(); i++)
    {        
        itKin = k.find(Data->KinNames.at(i));
        if(itKin != k.end())
        {
            Kinematics.push_back(itKin->second);
        }
        else
        {
            CMN_LOG_CLASS_RUN_ERROR << "LookupStateData: Kinematics object \"" << Data->KinNames.at(i) << " - "  << i << "\" not found" << std::endl;
            cmnThrow("LookupStateData: Kinematics object \"" + Data->KinNames.at(i) + "\" not found");
        }
    }

    //find sensor objects by name, update pointers
    Sensors.clear();
    for(size_t i = 0; i < Data->SensorNames.size(); i++)
    {
        itSen = s.find(Data->SensorNames.at(i));
        if(itSen != s.end())
        {
            Sensors.push_back(itSen->second);
        }
        else
        {
            CMN_LOG_CLASS_RUN_ERROR << "LookupStateData: Sensor object \"" << Data->SensorNames.at(i) << "\" not found" << std::endl;
            cmnThrow("LookupStateData: Sensor object \"" + Data->SensorNames.at(i) + "\" not found");
        }
    }
}

//! Updates internal references with co tableau.
/*! UpdateTableauRefs
\param co The control optimizer object
*/
void mtsVFBase::SetTableauRefs(nmrConstraintOptimizer & co)
{
    //assign the VF's refs to optimizer by passing them all to the optimizer
    co.SetRefs(Data->ObjectiveRows,Data->IneqConstraintRows,Data->EqConstraintRows,
               Data->NumSlacks,ObjectiveMatrixRef,ObjectiveMatrixSlackRef,ObjectiveVectorRef,
               IneqConstraintMatrixRef,IneqConstraintMatrixSlackRef,IneqConstraintVectorRef, IneqConstraintVectorSlackRef,
               EqConstraintMatrixRef,EqConstraintVectorRef);
}

vctDoubleMat mtsVFBase::Skew(const vctDoubleVec &in)
{
    vctDoubleMat out(3,3);
    out.SetAll(0.0);
    out[0][1] = -in[2];
    out[0][2] = in[1];
    out[1][0] = in[2];
    out[1][2] = -in[0];
    out[2][0] = -in[1];
    out[2][1] = in[0];
    return out;
}
