cmake_minimum_required(VERSION 2.8.3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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

    # find pacage
    find_package (sawConstraintController REQUIRED)
    include_directories(${sawConstraintController_INCLUDE_DIR})
    message(${sawConstraintController_INCLUDE_DIR})
    link_directories(${sawConstraintController_LIBRARY_DIR})
    message(${sawConstraintController_LIBRARY_DIR})

    # include directories
    include_directories (${sawConstraintController_INCLUDE_DIR})
    set(HEADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    message(${HEADER_DIR})

    # set header/source files
    set(HEADER_FILES
            ${HEADER_DIR}/CSABase.h
            ${HEADER_DIR}/simpleRobot.h)
    set(SOURCE_FILES
            main.cpp
            CSABase.cpp
            simpleRobot.cpp)

    # add executable
    add_executable(csaConstraintController
            ${HEADER_FILES}
            ${SOURCE_FILES})

    # link libraries
    cisst_target_link_libraries(csaConstraintController ${REQUIRED_CISST_LIBRARIES})
    target_link_libraries(csaConstraintController ${sawConstraintController_LIBRARIES})

endif(cisst_FOUND)
