#
# (C) Copyright 2019-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# Author: Max zhaoshuo Li
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(saw_constraint_controller_examples)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        cisst_ros_bridge
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
)

# cisst packages
set (REQUIRED_CISST_LIBRARIES
        cisstCommon
        cisstVector
        cisstMesh
        cisstMultiTask
        cisstNumerical
        cisstParameterTypes
        cisstOSAbstraction
)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)
    # load cisst configuration
    include (${CISST_USE_FILE})

    # find include in catkin space
    include_directories (${catkin_INCLUDE_DIRS})
    file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
    catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  CATKIN_DEPENDS cisst_ros_bridge geometry_msgs roscpp std_msgs)

    # find pacage
    find_package (sawConstraintController REQUIRED)
    include_directories(${sawConstraintController_INCLUDE_DIR})
    link_directories(${sawConstraintController_LIBRARY_DIR})

    # add executable
    add_executable(simpleCoop
            code/main_coop.cpp
            code/simple_coop.h
            code/simple_coop.cpp)
    add_executable(simpleTeleop
            code/main_teleop.cpp
            code/simple_teleop.h
            code/simple_teleop.cpp)

    # link libraries
    cisst_target_link_libraries(simpleCoop ${REQUIRED_CISST_LIBRARIES})
    target_link_libraries(simpleCoop
            ${sawConstraintController_LIBRARIES}
            ${catkin_LIBRARIES})

    cisst_target_link_libraries(simpleTeleop ${REQUIRED_CISST_LIBRARIES})
    target_link_libraries(simpleTeleop
            ${sawConstraintController_LIBRARIES}
            ${catkin_LIBRARIES})

endif(cisst_FOUND)
