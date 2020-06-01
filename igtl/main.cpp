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
#include <sawSensablePhantom/mtsSensableHD.h>
#include <sawSensablePhantom/mtsSensableHDQtWidget.h>
#include <sawOpenIGTLink/mtsIGTLCRTKBridge.h>

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

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    std::list<std::string> managerConfig;
    double igtlPeriodInSeconds = 10.0 * cmn_ms;
    int port = 18944;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonConfigFile);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");
    options.AddOptionOneValue("p", "period",
                              "IGTL send period in seconds (default is 10ms i.e. 0.01s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &port);
    options.AddOptionOneValue("P", "port",
                              "IGTL port (default is 18944)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &port);

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

    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();

    // create the components
    mtsSensableHD * devices = new mtsSensableHD("SensableHD");
    devices->Configure(jsonConfigFile);
    componentManager->AddComponent(devices);

    // IGTL bridge
    mtsIGTLCRTKBridge * igtlBridge = new mtsIGTLCRTKBridge("SensableHD-igtl", igtlPeriodInSeconds);
    igtlBridge->SetPort(port);
    componentManager->AddComponent(igtlBridge);

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;
    mtsSensableHDQtWidget * deviceWidget;

    // configure all components
    std::vector<std::string> deviceNames;
    deviceNames = devices->DeviceNames(); // get names of all sensable devices
    for (size_t index = 0;
         index < deviceNames.size();
         ++index) {
        std::string name = deviceNames.at(index);
        // Qt
        deviceWidget = new mtsSensableHDQtWidget(name + "-gui");
        deviceWidget->Configure();
        componentManager->AddComponent(deviceWidget);
        componentManager->Connect(deviceWidget->GetName(), "Device",
                                  devices->GetName(), name);
        tabWidget->addTab(deviceWidget, name.c_str());
        // igtl
        igtlBridge->BridgeInterfaceProvided(devices->GetName(),
                                            name, name);
    }

    // connect the igtl bridge(s)
    igtlBridge->Connect();

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

    return 0;
}
