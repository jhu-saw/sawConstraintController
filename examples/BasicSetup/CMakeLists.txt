#
# (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstNumerical
     cisstMultiTask
     cisstParameterTypes
     cisstRobot)

find_package (cisst REQUIRED)

if (cisst_FOUND)
  # Modify CMake configuration to use cisst
  include (${CISST_USE_FILE})

    find_package (sawConstraintController REQUIRED)

    if (sawConstraintController_FOUND)
        include_directories (${sawConstraintControllerExampleBasicSetup_SOURCE_DIR}
                             ${sawConstraintController_INCLUDE_DIR})
        link_directories (${sawConstraintController_LIBRARY_DIR})

        add_executable (sawConstraintControllerExampleBasicSetup
                        main.cpp
                        mtsRobotTask.cpp
                        mtsRobotTask.h
                        )

        target_link_libraries (sawConstraintControllerExampleBasicSetup
                               ${sawConstraintController_LIBRARIES})

        cisst_target_link_libraries (sawConstraintControllerExampleBasicSetup ${REQUIRED_CISST_LIBRARIES})

        set_property (TARGET sawConstraintControllerExampleBasicSetup PROPERTY FOLDER "sawConstraintController")

    endif (sawConstraintController_FOUND)

endif (cisst_FOUND)
