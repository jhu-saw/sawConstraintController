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
  set (sawConstraintController_INCLUDE_DIR "${sawConstraintController_SOURCE_DIR}/include")
  set (sawConstraintController_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawConstraintController_LIBRARIES sawConstraintController)

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
			mtsVFDataSideview.cdg
			mtsVFDataCartesianTranslation.cdg
                        prmJointState.cdg
                        mtsVFDataFollow.cdg
                        mtsVFDataPlane.cdg)

  include_directories (${sawConstraintController_INCLUDE_DIR})
  set (sawConstraintController_HEADER_DIR "${sawConstraintController_INCLUDE_DIR}/sawConstraintController")

  set (HEADER_FILES
       ${sawConstraintController_HEADER_DIR}/mtsVFBase.h
       ${sawConstraintController_HEADER_DIR}/mtsVFJointVel.h
       ${sawConstraintController_HEADER_DIR}/mtsVFJointPos.h
       ${sawConstraintController_HEADER_DIR}/mtsVFCartVel.h
       ${sawConstraintController_HEADER_DIR}/mtsVFCartOrientationVel.h
       ${sawConstraintController_HEADER_DIR}/mtsVFSensorCompliance.h
       ${sawConstraintController_HEADER_DIR}/mtsRobotTask.h
       ${sawConstraintController_HEADER_DIR}/mtsVFController.h
       ${sawConstraintController_HEADER_DIR}/mtsVFSideview.h
       ${sawConstraintController_HEADER_DIR}/mtsVFFollow.h
       ${sawConstraintController_HEADER_DIR}/mtsVFPlane.h
       )

  set (SOURCE_FILES
       mtsVFBase.cpp
       mtsVFJointVel.cpp
       mtsVFJointPos.cpp
       mtsVFCartVel.cpp
       mtsVFCartOrientationVel.cpp
       mtsVFSensorCompliance.cpp
       mtsRobotTask.cpp
       mtsVFController.cpp
       mtsVFSideview.cpp
       mtsVFFollow.cpp
       mtsVFPlane.cpp
       )

  add_library (sawConstraintController
               ${HEADER_FILES}
               ${SOURCE_FILES}
               ${sawConstraintController_CISST_DG_SRCS}
               ${sawConstraintController_CISST_DG_HDRS})
  cisst_target_link_libraries (sawConstraintController ${REQUIRED_CISST_LIBRARIES})
  set_property (TARGET sawConstraintController PROPERTY FOLDER "sawConstraintController")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawConstraintControllerConfig.cmake.in"
                  "${sawConstraintController_BINARY_DIR}/sawConstraintControllerConfig.cmake"
                  @ONLY@)

  # Install targets for headers and library
  install (DIRECTORY
           ${sawConstraintController_INCLUDE_DIR}/sawConstraintController
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawConstraintController
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawConstraintController_BINARY_DIR}/sawConstraintControllerConfig.cmake"
           DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

endif (cisst_FOUND)
