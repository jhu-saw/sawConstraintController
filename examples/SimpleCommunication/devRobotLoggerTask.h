// $Id

#ifndef _devRobotLoggerTask_h
#define _devRobotLoggerTask_h

#include <string>
#include <iostream>

#include <cisstMultiTask.h>
#include <cisstParameterTypes/prmRobotState.h>



//TODO add note taking scheme
//Write a function to do so here. and insert it into the OCT code later.
//add timestamp
class devRobotLoggerTask: public mtsTaskPeriodic {
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

protected:   

    //Temporary variable to get the latest state index
    mtsStateIndex StateIndex;
    //this is used to note the last logged stateIndex.
    mtsStateIndex LastStateIndex;
    //Current File write itterator
    mtsUInt       NumWritten;

    //Command pattern place holders.
    //mtsFunctionRead GetRawValues;
    struct {
        mtsFunctionRead         GetStateIndex;

        //reads the wavelength corresponding to each pixel.
        mtsFunctionQualifiedRead GetForcesQR;
        mtsFunctionQualifiedRead GetForceNormQR;
        mtsFunctionQualifiedRead GetForceDirectionQR;
        mtsFunctionQualifiedRead GetRobotStateQR;
        mtsFunctionQualifiedRead GetHandleForcesQR;
        mtsFunctionQualifiedRead GetRobotPosCorrectedQR;
        mtsFunctionRead          GetRobotState;
        mtsFunctionQualifiedRead GetForceRawQR;
        mtsFunctionQualifiedRead GetPedalInputQR;

    	
    } Robot;
    
    mtsDoubleVec                TipForces;
    mtsDouble                   TipForceNorm;
    mtsDoubleVec                TipForceDirection;
    mtsDoubleVec                HandleForces;
    mtsDoubleVec                RobotPosCorrected;
    mtsDoubleVec                FBGRawValues;
    mtsDoubleVec                PedalInputs;

    prmRobotState               RobotState;


    //void addNote(string s);
    bool OpenFiles(const std::string &fileNameBase);
    void CloseFiles(void);

    //flags set by the user

    //functions that can be accessed via the command pattern
    void EnableLog();
    void DisableLog();

    std::ofstream   LogFileForces;
    std::ofstream   LogFileNotes;
    std::ofstream   LogFileReadme;

    std::string     FileNameBase;
    mtsBool         LogEnabled;

    char			Delim;

public:
    devRobotLoggerTask(const std::string & taskName, double period);
    ~devRobotLoggerTask();

    void WriteNote(const mtsStdString &note); //not used here
    void IsLogging(mtsBool &isLogging) const;
    void SetFileName(const mtsStdString & fileNameBase);
    void SetLogEnabled(const mtsBool &enable);


    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void) {};

};
CMN_DECLARE_SERVICES_INSTANTIATION(devRobotLoggerTask);
#endif // _devRobotLoggerTask_h
