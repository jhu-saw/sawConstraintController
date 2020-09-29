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

#ifndef _mtsVFBase_h
#define _mtsVFBase_h

#include <sawConstraintController/prmKinematicsState.h>
#include <sawConstraintController/prmSensorState.h>
#include <sawConstraintController/mtsVFDataBase.h>
#include <cisstNumerical/nmrConstraintOptimizer.h>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

//! This is the base class for all virtual fixture objects
/*! \brief mtsVFBase: A class that contains logic for the implementation of virtual fixtures
 */
class CISST_EXPORT mtsVFBase: public mtsGenericObject
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE)

public:

        enum CONTROLLERMODE {JPOS = 1, JVEL = 2, CARTPOS = 3, CARTVEL = 4};

    //! Name of the virtual fixture
    std::string Name;

    //! Internally stored data
    mtsVFDataBase * Data;

    //! Frame object
    std::vector<prmKinematicsState *> Kinematics;

    //! Sensor object
    std::vector<prmSensorState *> Sensors;

    static std::string DefaultKinematicsName;

    vctDoubleVec DOFSelections;

    //! Objective data reference
    vctDynamicMatrixRef<double> ObjectiveMatrixRef;

    //! Objective slack reference
    vctDynamicMatrixRef<double> ObjectiveMatrixSlackRef;

    //! Objective vector reference
    vctDynamicVectorRef<double> ObjectiveVectorRef;

    //! Inequality constraint data reference
    vctDynamicMatrixRef<double> IneqConstraintMatrixRef;

    //! Inequality constraint slack reference
    vctDynamicMatrixRef<double> IneqConstraintMatrixSlackRef;

    //! Inequality constraint vector reference
    vctDynamicVectorRef<double> IneqConstraintVectorRef;

    //! Inequality constraint slack reference
    vctDynamicVectorRef<double> IneqConstraintVectorSlackRef;

    //! Equality constraint data reference
    vctDynamicMatrixRef<double> EqConstraintMatrixRef;

    //! Equality constraint slack reference
    vctDynamicMatrixRef<double> EqConstraintMatrixSlackRef;

    //! Equality constraint vector reference
    vctDynamicVectorRef<double> EqConstraintVectorRef;

public:

    /*! Constructor
     */
    mtsVFBase(){}

    /*! Constructor
      \param name String name of object
      \param ObjectiveRows Size of objective
      \param IneqConstraintRows Size of inequality constraint
      \param EqConstraintRows Size of equality constraint
    */
    mtsVFBase(const std::string & name, mtsVFDataBase * data);

    ~mtsVFBase() {
        delete Data;
    }

    //! Computes the necessary size of constraints, modifier the values in Data
    //! Overload if constraint size varies (such as mesh constraint)
    virtual void ComputeConstraintSize() {}

    //! Reserves space in the control optimizer.
    /*! reserve_space
      \param co Control optimizer object
    */
    void ReserveSpace(nmrConstraintOptimizer & co);

    //! Updates internal state data.
    /*! UpdateStateData
      \param k A map of names to kinematics objects
      \param s A map of names to sensor objects
    */
    void LookupStateData(const std::map<std::string, prmKinematicsState *> & k, const std::map<std::string, prmSensorState *> & s);

    //! Updates internal references with co tableau.
    /*! UpdateTableauRefs
      \param co The control optimizer object
    */
    void SetTableauRefs(nmrConstraintOptimizer & co);

    // This will be overloaded by subclasses that use this data in specific ways
    // For example, one can also take an osaSensorValue object relating to the force sensor
    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
     */
    virtual void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE Mode, const double TickTime) = 0;

    vctDoubleMat Skew(const vctDoubleVec & in);

    //! Converts the data in the references if a change in mode is needed
    virtual void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime) = 0;

    virtual void AssignRefs(const mtsVFBase::CONTROLLERMODE, const double, const vctDoubleVec & ,vctDoubleMat &, vctDoubleVec &, vctDoubleMat &, vctDoubleVec &, vctDoubleMat &, vctDoubleVec &){}

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFBase)

#endif // _mtsVFBase_h
