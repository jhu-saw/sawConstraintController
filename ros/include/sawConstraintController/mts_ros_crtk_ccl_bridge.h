/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Henry Phalen
  Created on: 2017-11-28

  (C) Copyright 2017-2021 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#ifndef _mts_ros_crtk_ccl_bridge_h
#define _mts_ros_crtk_ccl_bridge_h

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>
#include <typeindex>
#include <memory>

class mts_ros_crtk_ccl_bridge: public mts_ros_crtk_bridge_provided
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    inline mts_ros_crtk_ccl_bridge(const std::string & _component_name,
                                   ros::NodeHandle * _node_handle,
                                   const double _period_in_seconds = 5.0 * cmn_ms):
        mts_ros_crtk_bridge_provided(_component_name, _node_handle, _period_in_seconds)
    {}
    
    std::map<std::string, std::type_index> bridge_map;

    inline ~mts_ros_crtk_ccl_bridge() {}

    /*! Everything needed to bridge the mtsSARobSysTask component */
    void bridge(const std::string & _component_name,
                const std::string & _interface_name,
                const double _publish_period_in_seconds,
                const double _tf_period_in_seconds);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mts_ros_crtk_ccl_bridge);

#endif // _mts_ros_crtk_ccl_bridge_h
