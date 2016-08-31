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

project (sawConstraintController)

cmake_minimum_required (VERSION 2.8)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical
     )

find_package (cisst 1.0.8 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawConstraintController_IS_CATKIN_BUILT)
  if (sawConstraintController_IS_CATKIN_BUILT)
    set (sawConstraintController_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/share/sawConstraintController/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawConstraintController_BINARY_DIR}/lib")
  else ()
    set (sawConstraintController_CONFIG_FILE_DIR "${sawConstraintController_BINARY_DIR}")
  endif ()

  # create/configure file for find_package (sawConstraintController)
  set (sawConstraintController_INCLUDE_DIR
       "${sawConstraintController_SOURCE_DIR}/include"
       "${sawConstraintController_BINARY_DIR}/include")
  set (sawConstraintController_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawConstraintController_LIBRARIES sawConstraintController)

  # Set the version number
  set (sawConstraintController_VERSION_MAJOR "1")
  set (sawConstraintController_VERSION_MINOR "1")
  set (sawConstraintController_VERSION_PATCH "0")
  set (sawConstraintController_VERSION "${sawConstraintController_VERSION_MAJOR}.${sawConstraintController_VERSION_MINOR}.${sawConstraintController_VERSION_PATCH}")

  # Generate sawConstraintControllerRevision.h
  configure_file ("${sawConstraintController_SOURCE_DIR}/code/sawConstraintControllerRevision.h.in"
                  "${sawConstraintController_BINARY_DIR}/include/sawConstraintController/sawConstraintControllerRevision.h")

  # create data type using the data generator
  cisst_data_generator (sawConstraintController
                        "${sawConstraintController_BINARY_DIR}/include" # where to save the file
                        "sawConstraintController/"    # sub directory for include
                        code/mtsVFDataBase.cdg
                        code/mtsVFDataSensorCompliance.cdg
                        code/prmKinematicsOffsetState.cdg
                        code/prmForceOffsetState.cdg
                        code/prmKinematicsState.cdg
                        code/prmOffsetState.cdg
                        code/prmSensorState.cdg
                        code/mtsVFDataCartesianTranslation.cdg
                        code/prmJointState.cdg
                        code/mtsVFDataPlane.cdg
                        code/mtsVFDataJointLimits.cdg
                        code/mtsVFDataAbsoluteJointLimits.cdg
                        code/mtsVFDataRCM.cdg)

  include_directories (${sawConstraintController_INCLUDE_DIR})
  set (sawConstraintController_HEADER_DIR "include/sawConstraintController")

  set (HEADER_FILES
       ${sawConstraintController_HEADER_DIR}/mtsVFBase.h
       ${sawConstraintController_HEADER_DIR}/mtsVFJointVel.h
       ${sawConstraintController_HEADER_DIR}/mtsVFJointPos.h
       ${sawConstraintController_HEADER_DIR}/mtsVFCartVel.h
       ${sawConstraintController_HEADER_DIR}/mtsVFCartOrientationVel.h
       ${sawConstraintController_HEADER_DIR}/mtsVFSensorCompliance.h
       ${sawConstraintController_HEADER_DIR}/mtsVFController.h
       ${sawConstraintController_HEADER_DIR}/mtsVFPlane.h
       ${sawConstraintController_HEADER_DIR}/mtsVFJointLimits.h
       ${sawConstraintController_HEADER_DIR}/mtsVFAbsoluteJointLimits.h
       ${sawConstraintController_HEADER_DIR}/mtsVFFollow.h
       ${sawConstraintController_HEADER_DIR}/mtsVF_RCM.h
       )

  set (SOURCE_FILES
       code/mtsVFBase.cpp
       code/mtsVFJointVel.cpp
       code/mtsVFJointPos.cpp
       code/mtsVFCartVel.cpp
       code/mtsVFCartOrientationVel.cpp
       code/mtsVFSensorCompliance.cpp
       code/mtsVFController.cpp
       code/mtsVFPlane.cpp
       code/mtsVFJointLimits.cpp
       code/mtsVFAbsoluteJointLimits.cpp
       code/mtsVFFollow.cpp
       code/mtsVF_RCM.cpp
       )

  add_library (sawConstraintController
               ${HEADER_FILES}
               ${SOURCE_FILES}
               ${sawConstraintController_CISST_DG_SRCS}
               ${sawConstraintController_CISST_DG_HDRS})

  set_property (TARGET sawConstraintController PROPERTY FOLDER "sawConstraintController")
  cisst_target_link_libraries (sawConstraintController ${REQUIRED_CISST_LIBRARIES})

  set (sawConstraintController_CMAKE_CONFIG_FILE "${sawConstraintController_CONFIG_FILE_DIR}/sawConstraintControllerConfig.cmake")
  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawConstraintControllerConfig.cmake.in"
                  "${sawConstraintController_CMAKE_CONFIG_FILE}"
                  @ONLY)

  cisst_add_config_version ("${sawConstraintController_CONFIG_FILE_DIR}/sawConstraintControllerConfigVersion.cmake"
                            VERSION ${sawConstraintController_VERSION}
                            DESTINATION "share/sawConstraintController"
                            COMPONENT sawConstraintController)

  # Install targets for headers and library
  install (DIRECTORY
           "${sawConstraintController_SOURCE_DIR}/include/sawConstraintController"
           "${sawConstraintController_BINARY_DIR}/include/sawConstraintController"
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawConstraintController
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawConstraintController_CMAKE_CONFIG_FILE}"
           DESTINATION "share/sawConstraintController")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
