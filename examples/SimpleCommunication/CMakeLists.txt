#
# (C) Copyright 2014 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstNumerical
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical
     )

find_package (cisst REQUIRED)

if (cisst_FOUND)
  # Modify CMake configuration to use cisst
  include (${CISST_USE_FILE})

    find_package (sawConstraintController REQUIRED)

    if (sawConstraintController_FOUND)
        include_directories (${VFSimpleCommunicationExample_SOURCE_DIR}
                             ${sawConstraintController_INCLUDE_DIR})

        add_executable (VFSimpleCommunicationExample
                        main.cpp
                        mtsSimpleVFController.h
                        mtsSimpleVFController.cpp
                        mtsRobotTask.cpp
                        mtsRobotTask.h
                        mtsVFControllerTask.cpp
                        mtsVFControllerTask.h
                        mtsVFSenderTask.cpp
                        mtsVFSenderTask.h
                        prmSimpleRobotKinematicsState.cpp
                        prmSimpleRobotKinematicsState.h                        
                        )

        target_link_libraries (VFSimpleCommunicationExample
                               ${sawConstraintController_LIBRARIES})

        cisst_target_link_libraries (VFSimpleCommunicationExample ${REQUIRED_CISST_LIBRARIES})

        set_property (TARGET VFSimpleCommunicationExample PROPERTY FOLDER "sawConstraintController")

    endif (sawConstraintController_FOUND)

endif (cisst_FOUND)
