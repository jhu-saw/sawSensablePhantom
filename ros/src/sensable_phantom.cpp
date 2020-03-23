/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-03-21

  (C) Copyright 2017-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsSystemQtWidget.h>
#include <cisstParameterTypes/prmEventButtonQtWidget.h>
#include <cisstParameterTypes/prmStateRobotQtWidget.h>
#include <sawSensablePhantom/mtsSensableHD.h>

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsSensable", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    ros::init(argc, argv, "dvrk", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    double rosPeriod = 2.0 * cmn_ms;
    std::list<std::string> managerConfig;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.002, 2 ms, 500Hz).  There is no point to have a period higher than the device",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the components
    mtsSensableHD * device = new mtsSensableHD("SensableHD");
    device->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(device);

    // ROS bridge for publishers
    mtsROSBridge * pub_bridge = new mtsROSBridge("sensable_pub", rosPeriod, &rosNodeHandle);
    // separate thread to spin, i.e. subscribe and events
    mtsROSBridge * spin_bridge = new mtsROSBridge("sensable_spin", 0.1 * cmn_ms, &rosNodeHandle);
    spin_bridge->PerformsSpin(true);
    componentManager->AddComponent(pub_bridge);
    componentManager->AddComponent(spin_bridge);

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // configure all components
    std::vector<std::string> devices;
    devices = device->DeviceNames(); // get names of all sensable devices
    for (size_t index = 0;
         index < devices.size();
         ++index) {
        // ROS namespace
        std::string name = devices.at(index);
        std::string deviceNamespace = name + "/";
        std::replace(deviceNamespace.begin(), deviceNamespace.end(), ' ', '_');
        std::replace(deviceNamespace.begin(), deviceNamespace.end(), '-', '_');
        std::replace(deviceNamespace.begin(), deviceNamespace.end(), '.', '_');

        // motion commands
        pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (name, "measured_js",
             deviceNamespace + "measured_js");
        pub_bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (name, "measured_cp",
             deviceNamespace + "measured_cp");
        pub_bridge->AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
            (name, "measured_cv",
             deviceNamespace + "measured_cv");
        spin_bridge->AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>
            (name, "servo_cf",
             deviceNamespace + "servo_cf");

        // buttons
        spin_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (name + "Button1", "Button",
             deviceNamespace + "button_1");
        spin_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (name + "Button2", "Button",
             deviceNamespace + "button_2");

        // device state
        spin_bridge->AddSubscriberToCommandWrite<std::string, std_msgs::String>
            (name, "state_command",
             deviceNamespace + "state_command");
        spin_bridge->AddPublisherFromEventWrite<prmOperatingState, crtk_msgs::operating_state>
            (name, "operating_state",
             deviceNamespace + "operating_state");
        spin_bridge->AddServiceFromCommandRead<prmOperatingState, crtk_msgs::trigger_operating_state>
            (name, "operating_state",
             deviceNamespace + "operating_state");

        // messages
        spin_bridge->AddLogFromEventWrite(name, "Error",
                                          mtsROSEventWriteLog::ROS_LOG_ERROR);
        spin_bridge->AddLogFromEventWrite(name, "Warning",
                                          mtsROSEventWriteLog::ROS_LOG_WARN);
        spin_bridge->AddLogFromEventWrite(name, "Status",
                                          mtsROSEventWriteLog::ROS_LOG_INFO);

        // connect the bridge after all interfaces have been created
        componentManager->Connect(pub_bridge->GetName(), name,
                                  device->GetName(), name);
        componentManager->Connect(spin_bridge->GetName(), name,
                                  device->GetName(), name);
        componentManager->Connect(spin_bridge->GetName(), name + "Button1",
                                  device->GetName(), name + "Button1");
        componentManager->Connect(spin_bridge->GetName(), name + "Button2",
                                  device->GetName(), name + "Button2");

        // Qt Widgets

        // state (joint & cartesian)
        prmStateRobotQtWidgetComponent * stateWidget
            = new prmStateRobotQtWidgetComponent("Sensable-" + name + "-StateWidget");
        stateWidget->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
        stateWidget->Configure();
        componentManager->AddComponent(stateWidget);
        componentManager->Connect(stateWidget->GetName(), "Component",
                                  device->GetName(), name);
        tabWidget->addTab(stateWidget, QString(name.c_str()) + " state");

        // buttons
        prmEventButtonQtWidgetComponent * buttonsWidget
            = new prmEventButtonQtWidgetComponent("Sensable-" + name + "-ButtonsWidget");
        buttonsWidget->AddEventButton(name + "Button1");
        buttonsWidget->AddEventButton(name + "Button2");
        componentManager->AddComponent(buttonsWidget);
        componentManager->Connect(buttonsWidget->GetName(), name + "Button1",
                                  device->GetName(), name + "Button1");
        componentManager->Connect(buttonsWidget->GetName(), name + "Button2",
                                  device->GetName(), name + "Button2");
        tabWidget->addTab(buttonsWidget, QString(name.c_str()) + " buttons");

        // system info (timing and messages)
        mtsSystemQtWidgetComponent * systemWidget
            = new mtsSystemQtWidgetComponent("Sensable-" + name + "-SystemWidget");
        systemWidget->Configure();
        componentManager->AddComponent(systemWidget);
        componentManager->Connect(systemWidget->GetName(), "Component",
                                  device->GetName(), name);
        tabWidget->addTab(systemWidget, QString(name.c_str()) + " system");
    }

    // custom user components
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    cmnLogger::Kill();

    // stop ROS node
    ros::shutdown();

    return 0;
}
