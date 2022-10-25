#
# (C) Copyright 2014-2022 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawConstraintController VERSION 1.1.0)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstMesh
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstNumerical
     )

find_package (cisst 1.1.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawConstraintController)
  set (sawConstraintController_INCLUDE_DIR
       "${sawConstraintController_SOURCE_DIR}/include"
       "${sawConstraintController_BINARY_DIR}/include")
  set (sawConstraintController_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawConstraintController_LIBRARIES sawConstraintController)

  # Generate sawConstraintControllerRevision.h
  cisst_add_config_files (sawConstraintController)

  # create data type using the data generator
  set (CDG_FILES
       code/cdg/prmKinematicsOffsetState.cdg
       code/cdg/prmForceOffsetState.cdg
       code/cdg/prmKinematicsState.cdg
       code/cdg/prmOffsetState.cdg
       code/cdg/prmSensorState.cdg
       code/cdg/mtsVFDataBase.cdg
       code/cdg/mtsVFDataSensorCompliance.cdg
       code/cdg/mtsVFDataCartesianTranslation.cdg
       code/cdg/mtsVFDataPlane.cdg
       code/cdg/mtsVFDataJointLimits.cdg
       code/cdg/mtsVFDataAbsoluteJointLimits.cdg
       code/cdg/mtsVFDataRCM.cdg
       code/cdg/mtsVFDataCylinder.cdg
       code/cdg/mtsVFDataMesh.cdg
       )

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
       ${sawConstraintController_HEADER_DIR}/mtsVFCartesianLimits.h
       ${sawConstraintController_HEADER_DIR}/mtsVFFollow.h
       ${sawConstraintController_HEADER_DIR}/mtsVFLimitsConstraint.h
       ${sawConstraintController_HEADER_DIR}/mtsVFRCM.h
       ${sawConstraintController_HEADER_DIR}/mtsVFCylinder.h
       ${sawConstraintController_HEADER_DIR}/mtsVFMesh.h
       ${sawConstraintController_HEADER_DIR}/mtsConstraintBase.h
       ${sawConstraintController_HEADER_DIR}/constraintControllerLite.h
       ${sawConstraintController_HEADER_DIR}/mtsConstraintFollowPositionTraj.h
       ${sawConstraintController_HEADER_DIR}/mtsConstraintAbsoluteJointLimits.h
       ${sawConstraintController_HEADER_DIR}/mtsConstraintFixOrientation.h
       ${sawConstraintController_HEADER_DIR}/mtsConstraintJointVelLimits.h
       ${sawConstraintController_HEADER_DIR}/mtsConstraintStayOnAxis.h
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
      code/mtsVFCartesianLimits.cpp
      code/mtsVFFollow.cpp
      code/mtsVFLimitsConstraint.cpp
      code/mtsVFRCM.cpp
      code/mtsVFCylinder.cpp
      code/mtsVFMesh.cpp
      code/ccl_helper.cpp
      code/constraintControllerLite.cpp
      code/mtsConstraintFollowPositionTraj.cpp
      code/mtsConstraintAbsoluteJointLimits.cpp
      code/mtsConstraintFixOrientation.cpp
      code/mtsConstraintJointVelLimits.cpp
      code/mtsConstraintStayOnAxis.cpp
      )

  cisst_data_generator (sawConstraintController
                      "${sawConstraintController_BINARY_DIR}/include" # where to save the file
                      "sawConstraintController/"    # sub directory for include
                      ${CDG_FILES}
                      )

  add_library (sawConstraintController
               ${HEADER_FILES}
               ${SOURCE_FILES}
               ${sawConstraintController_CISST_DG_SRCS}
               ${sawConstraintController_CISST_DG_HDRS})

  set_target_properties (sawConstraintController PROPERTIES
                         VERSION ${sawConstraintController_VERSION}
                         FOLDER "sawConstraintController")
  cisst_target_link_libraries (sawConstraintController ${REQUIRED_CISST_LIBRARIES})

  # Install targets for headers and library
  install (DIRECTORY
           "${sawConstraintController_SOURCE_DIR}/include/sawConstraintController"
           "${sawConstraintController_BINARY_DIR}/include/sawConstraintController"
           DESTINATION include
	   COMPONENT sawConstraintController-dev
           PATTERN .svn EXCLUDE)

  install (TARGETS sawConstraintController COMPONENT sawConstraintController
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
