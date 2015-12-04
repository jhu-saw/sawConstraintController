#
# (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

project (sawConstraintController)

########################### Misc. Configs ##############################
# Add custom cmake files folder
SET (PROJECT_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# This sets where to look for modules (e.g. "Find*.cmake" files)
SET (CMAKE_MODULE_PATH "${PROJECT_CMAKE_DIR}" "${CMAKE_MODULE_PATH}" )


# Add custom Install Settings
include(InstallSettings)

################ Installation project target ################

set(TARGET_NAME ${PROJECT_NAME}Targets)
add_custom_target(${TARGET_NAME} ALL)
include(GenerateExportHeader)

add_subdirectory (code)
add_subdirectory (examples)



######################### Installation Stuff ###########################


include(InstallProjectConfig)

###################### Add uninstall target ############################
if("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    ADD_CUSTOM_TARGET(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${PROJECT_CMAKE_DIR}/cmake_uninstall.cmake)
endif()


##################### Show Configuration Summary ######################
MESSAGE(STATUS "===============================================================")
MESSAGE(STATUS "================  Configuration Summary  ======================")
MESSAGE(STATUS "Project Name:       ${PROJECT_NAME}")
MESSAGE(STATUS "Build type:         ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Build type Flags:   ${CMAKE_BUILD_TYPE_FLAGS}")
MESSAGE(STATUS "C++ compile flags:  ${CMAKE_CXX_FLAGS}")
MESSAGE(STATUS "Install Path:       ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "Build Examples:     ${BUILD_EXAMPLES}")