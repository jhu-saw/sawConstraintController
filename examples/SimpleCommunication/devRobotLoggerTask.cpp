/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
Author(s):	Marcin Balicki
Created on:   2008-09-03

(C) Copyright 2008 Johns Hopkins University (JHU), All Rights
Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifdef _MSC_VER
#pragma warning(disable:4786)
#pragma warning(disable:4996)
#endif

#include "devRobotLoggerTask.h"
#include <cisstOSAbstraction/osaGetTime.h>

#include <math.h>
#include <iomanip>
#include <time.h>

//#include <direct.h>
//for linux please see www.brainbell.com/tutorials/c++/Creating_A_Directory.htm

CMN_IMPLEMENT_SERVICES(devRobotLoggerTask);

devRobotLoggerTask::devRobotLoggerTask(const std::string & taskName,
                                       double period):
mtsTaskPeriodic(taskName, period, false, 1000)
{
    Delim= ',';

    //to access the whole vector use .Data
    //this gets resized by the sensor class anyway.
    TipForces.SetSize(3);
    TipForces.Zeros();
    TipForceNorm = 0;
    TipForceDirection.SetSize(1); //will get resized by fbgsensor
    TipForceDirection.Zeros(); //will get resized by fbgsensor

    FBGRawValues.SetSize(3);
    FBGRawValues.Zeros();

    FileNameBase=std::string("");
    LastStateIndex = mtsStateIndex(0,0,0,1000);

    NumWritten=0;
    LogEnabled=false;

    StateTable.AddData(LastStateIndex,  "LastSavedIndex");
    StateTable.AddData(NumWritten,      "NumSamplesWritten");
    StateTable.AddData(LogEnabled,      "LogEnabled");

    mtsInterfaceProvided *logInterface = AddInterfaceProvided("ProvidesRobotLogger");

    if (logInterface){
        logInterface->AddCommandReadState(StateTable,LastStateIndex,                "GetLastSavedIndex");
        logInterface->AddCommandReadState(StateTable,NumWritten  ,                  "GetNumSamplesWritten");
        logInterface->AddCommandReadState(StateTable,LogEnabled  ,                  "GetLogEnabled");
        logInterface->AddCommandWrite(&devRobotLoggerTask::SetLogEnabled ,       this,    "SetLogEnabled", mtsBool());
        logInterface->AddCommandWrite(&devRobotLoggerTask::SetFileName,          this,    "SetFileName", mtsStdString());
        logInterface->AddCommandWrite(&devRobotLoggerTask::WriteNote,            this,    "WriteNote",   mtsStdString());
    }
    //oper a resource port that will connect to the robot interface.
    mtsInterfaceRequired * robotInterface = AddInterfaceRequired("RequiresEyeRobot");

    if (robotInterface) {

        robotInterface->AddFunction("GetTableIndex",        Robot.GetStateIndex);
        robotInterface->AddFunction("GetFBGForces",         Robot.GetForcesQR);
        robotInterface->AddFunction("GetFBGForceNorm",      Robot.GetForceNormQR);
        robotInterface->AddFunction("GetFBGForceDirection", Robot.GetForceDirectionQR);
        robotInterface->AddFunction("GetFBGForceRaw",       Robot.GetForceRawQR);

        robotInterface->AddFunction("GetRobotState",	    Robot.GetRobotStateQR);
        robotInterface->AddFunction("GetHandleSensor",      Robot.GetHandleForcesQR);
        robotInterface->AddFunction("GetRobotPosCorrected", Robot.GetRobotPosCorrectedQR);
        robotInterface->AddFunction("GetPedalInput",        Robot.GetPedalInputQR);

    }
}
devRobotLoggerTask::~devRobotLoggerTask ( ){
    CloseFiles();
}

void  devRobotLoggerTask::SetLogEnabled(const mtsBool &enable){

    //if we not already logging then we should reset the latest and start.
    if(!LogEnabled.Data){
        Robot.GetStateIndex(LastStateIndex);
    }
    LogEnabled=enable;
}

void  devRobotLoggerTask::WriteNote(const mtsStdString &note){

    osaAbsoluteTime absTime;
    mtsTaskManager::GetInstance()->GetTimeServer().RelativeToAbsolute(note.Timestamp(),absTime);

    LogFileNotes<<std::setprecision(6)<< absTime.ToSeconds() << " ; " << note.Data << std::endl;

}

void devRobotLoggerTask::Startup(void)
{
    //this zeros the state index...it might miss a few initial values but
    //generally that is ok.
    //LastStateIndex.set=0;
    Robot.GetStateIndex(LastStateIndex);
    //Robot.GetRobotState(this->RobotState);

    //get the values for the wavelength here.
    // open all files and set some initial data.
    if(!OpenFiles(FileNameBase)){
        CMN_LOG_CLASS_INIT_ERROR << "Startup: can't open files" << std::endl;
        //exit(-1);
    }

}

void devRobotLoggerTask::Run(void)
{
    //default
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    osaAbsoluteTime   absTime;

    if (!LogEnabled){
        return;
    }

    Robot.GetStateIndex(StateIndex);
    static unsigned int statesToSave = 0;
    statesToSave = StateIndex.Ticks() - LastStateIndex.Ticks();
    static mtsStateIndex tempIndex;

    //if difference is greater then the circular buffer,
    //then get 3/4 of the
    //this is hack to make sure we don't overflow.
    if (statesToSave > 750){
        statesToSave = 750;
        CMN_LOG_CLASS_INIT_ERROR << "Log Error, too many states to save!" << std::endl;
    }
    while (statesToSave>0){
        statesToSave--;
        tempIndex=StateIndex-statesToSave;

        //get all the relevent data.
        Robot.GetForcesQR(tempIndex,          TipForces);
        Robot.GetForceNormQR(tempIndex,       TipForceNorm);
        Robot.GetForceDirectionQR(tempIndex,  TipForceDirection);
        Robot.GetHandleForcesQR(tempIndex,    HandleForces);
        Robot.GetRobotStateQR(tempIndex,      RobotState);
        Robot.GetForceRawQR(tempIndex,        FBGRawValues);   // !todo this should be removed when done with the experiment.
        Robot.GetPedalInputQR(tempIndex,      PedalInputs);
        Robot.GetRobotPosCorrectedQR(tempIndex,    RobotPosCorrected);

        mtsTaskManager::GetInstance()->GetTimeServer().RelativeToAbsolute(RobotState.Timestamp(),absTime);


        LogFileForces<<std::setprecision(0)<<tempIndex.Ticks()
               <<Delim<<std::setiosflags(std::ios::fixed)
               <<std::setprecision(6)<<absTime.ToSeconds();

        LogFileForces<<Delim<<std::setprecision(0)<<TipForces.Valid()
                <<Delim<<std::setprecision(7);

        unsigned int i = 0;
        for (i = 0;i < TipForces.size();i++)
            LogFileForces<<TipForces[i]<<Delim;
        //
        LogFileForces<<TipForceNorm.Data;
        for (i = 0;i < TipForceDirection.size();i++)
            LogFileForces<<Delim<<TipForceDirection[i];

        for (i = 0;i < FBGRawValues.size();i++)
            LogFileForces<<Delim<<FBGRawValues[i];


        for (i = 0;i < HandleForces.size();i++)
            LogFileForces<<Delim<<HandleForces[i];

        LogFileForces << std::setprecision(4);
        for (i = 0;i < RobotState.CartesianPosition().size();i++)
            LogFileForces<<Delim<<RobotState.CartesianPosition()[i];

        for (i = 0;i < RobotState.CartesianVelocity().size();i++)
            LogFileForces<<Delim<<RobotState.CartesianVelocity()[i];

        for (i = 0;i < RobotPosCorrected.size();i++)
            LogFileForces<<Delim<<RobotPosCorrected[i];

        for (i = 0;i < PedalInputs.size();i++)
            LogFileForces<<Delim<<PedalInputs[i];


        LogFileForces<<std::endl;
        NumWritten.Data++;

        //LogRawData.setf(ios::fixed); //Print floating point numbers using fixed point notation.
        ////ios::showpoint  Print a decimal point for all floating point numbers, even when
        ////it's not needed (e.g. the number is exactly an integer).
        ////The precision of numbers can be changed as follows. You can also set the width,
        ////i.e. the minimum number of spaces used to print the next.
        ////These featuers are used, for example, to make output items line up in columns
        ////when printed. Both of these features require that you include the iomanip header file.
        //LogRawData.precision(0);  // print 0 digits after decimal point
    }
    LastStateIndex = StateIndex;		//save for next time around.
    //save tha last known state index
}

bool devRobotLoggerTask::OpenFiles(const std::string &fileNameBase){

    CloseFiles();

    LogFileForces.clear();
    LogFileNotes.clear();
    //LogFileReadme.clear();

    std::string dateTime;
    osaGetDateTimeString(dateTime);
    std::string fileName;

    fileName=dateTime+std::string("-")+fileNameBase+std::string("-RobotLog")+std::string(".csv");
    LogFileForces.open(fileName.c_str(), std::ios::out | std::ios::app);

    fileName=dateTime+std::string("-")+fileNameBase+std::string("-RobotNotes")+std::string(".csv");
    LogFileNotes.open(fileName.c_str(), std::ios::out | std::ios::app);

    //TODO: add a file that describes all the files that are written
    //		should add the format of each file in there.
    //fileName=dirName+std::string("Log6Readme")+std::string(".txt");
    //LogFileReadme.open(fileName.c_str(), std::ios::out | std::ios::app);

    if (LogFileForces.fail() || LogFileNotes.fail() ){
        CMN_LOG_CLASS_INIT_ERROR << "Can't Open files: " << std::endl;
        return false;
    }
    else {
        //add header
        LogFileForces.precision(4);
        LogFileForces.setf(std::ios::fixed);

        LogFileNotes.precision(4);
        LogFileNotes.setf(std::ios::fixed);
        RobotPosCorrected.Zeros();

        Robot.GetForcesQR(LastStateIndex,               TipForces);
        Robot.GetForceNormQR(LastStateIndex,            TipForceNorm);
        Robot.GetForceDirectionQR(LastStateIndex,       TipForceDirection);
        Robot.GetHandleForcesQR(LastStateIndex,	        HandleForces);
        Robot.GetRobotStateQR(LastStateIndex,           RobotState);
        Robot.GetRobotPosCorrectedQR(LastStateIndex,    RobotPosCorrected);
        Robot.GetForceRawQR(LastStateIndex,             FBGRawValues);

        Robot.GetPedalInputQR(LastStateIndex,           PedalInputs);


        //double currentTime=mtsTaskManager::GetInstance()->GetTimeServer().GetRelativeTime();
        osaAbsoluteTime timeOrigin;
        mtsTaskManager::GetInstance()->GetTimeServer().GetTimeOrigin(timeOrigin);

        double timeOriginInSec = timeOrigin.sec + timeOrigin.nsec/1.0e9;

        //LogFileForces<<"Ticks"<<Delim<<"TimeStamp(origin:"<<timeOriginInSec<<")"<<Delim<<"Valid"<<Delim;
        LogFileForces<<"# Ticks"<<Delim<<"TimeStamp"<<Delim<<"ValidTipForce"<<Delim;

        unsigned int i;
        for (i = 0; i < TipForces.size(); i++)
            LogFileForces<<"TipForce_Nm_"<<i<<Delim;
        //
        LogFileForces<<"TipForceNorm_Nm"<<Delim;

        for (i = 0;i < TipForceDirection.size(); i++)
            LogFileForces<<"TipForceDirection"<<i<<Delim;

        for (i = 0;i < FBGRawValues.size(); i++)
            LogFileForces<<"FBGRaw"<<i<<Delim;

        for (i = 0;i < HandleForces.size(); i++)
            LogFileForces<<"HandleForcesN"<<i<<Delim;

        for (i = 0;i < RobotState.CartesianPosition().size(); i++)
            LogFileForces<<"RobotPosition"<<i<<Delim;

        for (i = 0;i < RobotState.CartesianVelocity().size(); i++)
            LogFileForces<<"RobotVelocity"<<i<<Delim;

        for (i = 0;i < RobotPosCorrected.size(); i++)
            LogFileForces<<"RobotPosCorrected"<<i<<Delim;

        for (i = 0;i < PedalInputs.size()-1; i++)
            LogFileForces<<"Pedal"<<i<<Delim;

        LogFileForces<<"Pedal"<<i<<std::endl;


        CMN_LOG_CLASS_RUN_VERBOSE << "Log files opened"<<std::endl;
    }
    return true;
}


void devRobotLoggerTask::SetFileName(const mtsStdString & fileNameBase)
{
    FileNameBase = fileNameBase.Data;
    LogEnabled = false;
    CloseFiles();
    OpenFiles(FileNameBase);
}


void devRobotLoggerTask::CloseFiles(void){
    LogFileForces.close();
    LogFileNotes.close();
    //LogFileReadme.close();
    NumWritten=0;
}

/*! I should include parameters for the axis here */
void devRobotLoggerTask::Configure(const std::string & fileName){
    CMN_LOG_CLASS_INIT_VERBOSE<< "Configuring Logger" << fileName << std::endl;
    //Open files used
}
