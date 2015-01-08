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

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical
     )

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # create/configure file for find_package (sawConstraintController)
  set (sawConstraintController_INCLUDE_DIR "${sawConstraintController_SOURCE_DIR}/include"
                                           "${sawConstraintController_BINARY_DIR}/include")
  set (sawConstraintController_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawConstraintController_LIBRARIES sawConstraintController)

  # Set the version number
  set (sawConstraintController_VERSION_MAJOR "1")
  set (sawConstraintController_VERSION_MINOR "0")
  set (sawConstraintController_VERSION_PATCH "0")
  set (sawConstraintController_VERSION "${sawConstraintController_VERSION_MAJOR}.${sawConstraintController_VERSION_MINOR}.${sawConstraintController_VERSION_PATCH}")

  # Generate sawConstraintControllerRevision.h
  configure_file ("${sawConstraintController_SOURCE_DIR}/code/sawConstraintControllerRevision.h.in"
                  "${sawConstraintController_BINARY_DIR}/include/sawConstraintController/sawConstraintControllerRevision.h")

  # create data type using the data generator
  cisst_data_generator (sawConstraintController
                        "${CISST_BINARY_DIR}/include" # where to save the file
                        "sawConstraintController/"    # sub directory for include
                        mtsVFDataBase.cdg
                        mtsVFDataSensorCompliance.cdg
                        prmKinematicsOffsetState.cdg
                        prmForceOffsetState.cdg
                        prmKinematicsState.cdg
                        prmOffsetState.cdg
                        prmSensorState.cdg			
			mtsVFDataCartesianTranslation.cdg
                        prmJointState.cdg                        
                        mtsVFDataPlane.cdg
                        mtsVFDataJointLimits.cdg)

  include_directories (${sawConstraintController_INCLUDE_DIR})
  set (sawConstraintController_HEADER_DIR "${sawConstraintController_SOURCE_DIR}/include/sawConstraintController")

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
       )

  set (SOURCE_FILES
       mtsVFBase.cpp
       mtsVFJointVel.cpp
       mtsVFJointPos.cpp
       mtsVFCartVel.cpp
       mtsVFCartOrientationVel.cpp
       mtsVFSensorCompliance.cpp
       mtsVFController.cpp             
       mtsVFPlane.cpp
       mtsVFJointLimits.cpp
       )

  add_library (sawConstraintController
               ${HEADER_FILES}
               ${SOURCE_FILES}
               ${sawConstraintController_CISST_DG_SRCS}
               ${sawConstraintController_CISST_DG_HDRS})
  cisst_target_link_libraries (sawConstraintController ${REQUIRED_CISST_LIBRARIES})
  set_property (TARGET sawConstraintController PROPERTY FOLDER "sawConstraintController")

  cisst_is_catkin_build (sawConstraintController_IS_CATKIN_BUILT)
  if (sawConstraintController_IS_CATKIN_BUILT)
    set (sawConstraintController_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/cmake")
  else ()
    set (sawConstraintController_CONFIG_FILE_DIR "${sawConstraintController_BINARY_DIR}")
  endif ()

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawConstraintControllerConfig.cmake.in"
                  "${sawConstraintController_CONFIG_FILE_DIR}/sawConstraintControllerConfig.cmake"
                  @ONLY@)

  cisst_add_config_version ("${sawConstraintController_CONFIG_FILE_DIR}/sawConstraintControllerConfigVersion.cmake"
                            VERSION ${sawConstraintController_VERSION}
                            DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
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

  install (FILES "${sawConstraintController_BINARY_DIR}/sawConstraintControllerConfig.cmake"
           DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

endif (cisst_FOUND)
