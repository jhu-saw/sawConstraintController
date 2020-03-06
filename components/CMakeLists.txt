#
# (C) Copyright 2014-2019 Johns Hopkins University (JHU), All Rights Reserved.
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
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_MESH "If mesh constraint is used, it depends on cisstICP" OFF)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes     
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

  # Generate sawConstraintControllerRevision.h
  configure_file ("${sawConstraintController_SOURCE_DIR}/code/sawConstraintControllerRevision.h.in"
                  "${sawConstraintController_BINARY_DIR}/include/sawConstraintController/sawConstraintControllerRevision.h")

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
)

if(USE_EIGEN)
    set (HEADER_FILES
        ${HEADER_FILES}
        ${sawConstraintController_HEADER_DIR}/constraintOptimizer.h
        )
    set (SOURCE_FILES
        ${SOURCE_FILES}
        code/eiquadprog.hpp
        )
    set(EIGEN3_INCLUDE_DIR "Eigen include directory" CACHE PATH "")
    include_directories(${EIGEN3_INCLUDE_DIR})
endif(USE_EIGEN)

# mesh depends on cisstICP
if (USE_MESH)
    message("Use mesh constraint")
    set (HEADER_FILES
        ${HEADER_FILES}
        ${sawConstraintController_HEADER_DIR}/mtsVFMesh.h
        )
    set (SOURCE_FILES
        ${SOURCE_FILES}
         code/mtsVFMesh.cpp
         )
    set(CDG_FILES
        ${CDG_FILES}
        code/cdg/mtsVFDataMesh.cdg
        )
    find_package(cisstICP)
    include_directories(${cisstICP_INCLUDE_DIR})
    link_directories(${cisstICP_LIBRARY_DIR})
endif(USE_MESH)

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

    set_property (TARGET sawConstraintController PROPERTY FOLDER "sawConstraintController")
    cisst_target_link_libraries (sawConstraintController ${REQUIRED_CISST_LIBRARIES})

# mesh depends on cisstICP
if(USE_MESH)
    target_link_libraries(sawConstraintController ${cisstICP_LIBRARIES})
endif(USE_MESH)

    set (sawConstraintController_CMAKE_CONFIG_FILE "${sawConstraintController_CONFIG_FILE_DIR}/sawConstraintControllerConfig.cmake")
    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawConstraintControllerConfig.cmake.in"
                  "${sawConstraintController_CMAKE_CONFIG_FILE}"
                  @ONLY)

    cisst_add_config_version ("${sawConstraintController_CONFIG_FILE_DIR}/sawConstraintControllerConfigVersion.cmake"
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
