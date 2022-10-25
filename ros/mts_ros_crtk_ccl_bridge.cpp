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

// system include
#include <iostream>
#include <typeinfo>

// cisst
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisst_ros_crtk/mts_ros_crtk_bridge.h>
#include <sawConstraintController/mts_ros_crtk_ccl_bridge.h>

CMN_IMPLEMENT_SERVICES(mts_ros_crtk_ccl_bridge);


void mts_ros_crtk_ccl_bridge::bridge(const std::string & _component_name,
                                     const std::string & _interface_name,
                                     const double _publish_period_in_seconds,
                                     const double _tf_period_in_seconds)
{
    // clean ROS namespace
    std::string _clean_namespace = _component_name;
    cisst_ros_crtk::clean_namespace(_clean_namespace);

    // controller specific topics, some might be CRTK compliant
    this->bridge_interface_provided(_component_name,
                                    _interface_name,
                                    _clean_namespace,
                                    _publish_period_in_seconds,
                                    _tf_period_in_seconds);

    // first make sure we can find the component to bridge
    mtsManagerLocal * _component_manager = mtsComponentManager::GetInstance();
    mtsComponent * _component = _component_manager->GetComponent(_component_name);
    if (!_component) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_interface_provided: unable to find component \""
                                 << _component_name << "\"" << std::endl;
        return;
    }
    // then try to find the interface
    mtsInterfaceProvided * _interface_provided = _component->GetInterfaceProvided(_interface_name);
    if (!_interface_provided) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_interface_provided: unable to find provided interface \""
                                 << _interface_name << "\" on component \""
                                 << _component_name << "\"" << std::endl;
        return;
    }

    
    // non CRTK topics
    // add trailing / for clean namespace
    if (!_clean_namespace.empty()) {
        _clean_namespace.append("/");
    }
    // required interfaces specific to this component to bridge
    const std::string _required_interface_name = _component_name + "_using_" + _interface_name;

    for(auto& command_name: _interface_provided->GetNamesOfCommandsWrite())
    {
        auto cmd_type_id_name = _interface_provided->GetCommandWriteArgumentServices(command_name)->TypeInfoPointer()->name();

        if (cmd_type_id_name == typeid(mtsGenericObjectProxy<bool>).name())
        {
            m_subscribers_bridge->AddSubscriberToCommandWrite<bool, std_msgs::Bool>
                (_required_interface_name, command_name, _clean_namespace + command_name);
        }
        else if (cmd_type_id_name == typeid(mtsGenericObjectProxy<double>).name())
        {
            m_subscribers_bridge->AddSubscriberToCommandWrite<double, std_msgs::Float64>
                (_required_interface_name, command_name, _clean_namespace + command_name);
        }

        else if ( cmd_type_id_name == typeid(mtsGenericObjectProxy<vctDoubleVec>).name() )
        {
            m_subscribers_bridge->AddSubscriberToCommandWrite<vctDoubleVec, std_msgs::Float64MultiArray>
                (_required_interface_name, command_name, _clean_namespace + command_name);
        }
        else if ( cmd_type_id_name == typeid(mtsGenericObjectProxy<vctRot3>).name() )
        {
            m_subscribers_bridge->AddSubscriberToCommandWrite<vctRot3, geometry_msgs::Quaternion>
                (_required_interface_name, command_name, _clean_namespace + command_name);
        }
        else if ( cmd_type_id_name == typeid(mtsGenericObjectProxy<vct3>).name() )
        {
             m_subscribers_bridge->AddSubscriberToCommandWrite<vct3, geometry_msgs::Vector3>
                (_required_interface_name, command_name, _clean_namespace + command_name);
        }
        else
        {
            std::cout << "No bridge found for command: " << command_name << " from provided interface: " << _interface_provided;
            std::cout  << "CommandWrite argument has type name: " << cmd_type_id_name;
            std::cout << " and class name: " <<  _interface_provided->GetCommandWriteArgumentServices(command_name)->GetName() << std::endl;
        }

    }

    // connections
    m_connections.Add(m_subscribers_bridge->GetName(), _required_interface_name,
                      _component_name, _interface_name);
    // m_connections.Add(m_events_bridge->GetName(), _required_interface_name,
    //                   _component_name, _interface_name);
}
