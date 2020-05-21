/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Anton Deguet
  Created on: 2008-04-04

  (C) Copyright 2008-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// Sensable headers
#include <HD/hd.h>
#include <HDU/hduError.h>

// Define this before including mtsTaskFromCallback.h (which is included by
// mtsSensableHD.h).
#define MTS_TASK_CALLBACK_CONVENTION HDCALLBACK
#include <sawSensablePhantom/mtsSensableHD.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmEventButton.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsSensableHD, mtsTaskFromCallback, mtsTaskConstructorArg);

struct mtsSensableHDDriverData {
    HDSchedulerHandle CallbackHandle;
    HDCallbackCode CallbackReturnValue;
};

struct mtsSensableHDHandle {
    HHD m_device_handle;
};

// internal data using Sensable data types
class mtsSensableHDDevice {

public:
    prmOperatingState m_operating_state;
    mtsFunctionWrite m_operating_state_event;

    inline void state_command(const std::string & command) {
        std::string humanReadableMessage;
        prmOperatingState::StateType newOperatingState;
        try {
            if (m_operating_state.ValidCommand(prmOperatingState::CommandTypeFromString(command),
                                               newOperatingState, humanReadableMessage)) {
                if (command == "enable") {
                    m_operating_state.State() = prmOperatingState::ENABLED;
                } else if (command == "disable") {
                    m_operating_state.State() = prmOperatingState::DISABLED;
                } else {
                    m_interface->SendStatus(this->m_name + ": state command \""
                                            + command + "\" is not supported yet");
                }
                // always emit event with current device state
                m_interface->SendStatus(this->m_name
                                        + ": current state is \""
                                        + prmOperatingState::StateTypeToString(m_operating_state.State()) + "\"");
                m_operating_state.Valid() = true;
                m_operating_state_event(m_operating_state);
            } else {
                m_interface->SendWarning(this->m_name + ": " + humanReadableMessage);
            }
        } catch (std::runtime_error & e) {
            m_interface->SendWarning(this->m_name + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
        }
    }

    inline void servo_cf(const prmForceCartesianSet & wrench) {
        if (m_operating_state.State() == prmOperatingState::ENABLED) {
            m_servo_cf = wrench;
        }
    }

    inline void get_button_names(std::list<std::string> & placeholder) const {
        placeholder.clear();
        placeholder.push_back(m_name + "Button1");
        placeholder.push_back(m_name + "Button2");
    }

    mtsInterfaceProvided * m_interface;

    bool m_device_available;

    // local copy of the buttons state as defined by Sensable
    int m_buttons;

    // local copy of the position and velocities
    prmPositionCartesianGet m_measured_cp;
    prmVelocityCartesianGet m_measured_cv;
    prmForceCartesianGet m_measured_cf;
    prmStateJoint m_measured_js;
    prmConfigurationJoint m_configuration_js;
    vctDynamicVectorRef<double> GimbalPositionJointRef, GimbalEffortJointRef;

    // mtsFunction called to broadcast the event
    mtsFunctionWrite m_button1_event, m_button2_event, m_state_event;

    prmForceCartesianSet m_servo_cf;

    // local buffer used to store the position as provided
    // by Sensable
    typedef vctFixedSizeMatrix<double, 4, 4, VCT_COL_MAJOR> Frame4x4Type;
    Frame4x4Type Frame4x4;
    vctFixedSizeConstVectorRef<double, 3, Frame4x4Type::ROWSTRIDE> Frame4x4TranslationRef;
    vctFixedSizeConstMatrixRef<double, 3, 3,
                               Frame4x4Type::ROWSTRIDE, Frame4x4Type::COLSTRIDE> Frame4x4RotationRef;

    std::string m_name;
};


// The task Run method
void mtsSensableHD::Run(void)
{
    int current_buttons;
    const size_t nbDevices = m_devices.size();
    mtsSensableHDDevice * device;
    mtsSensableHDHandle * handle;
    HHD hHD;

    for (size_t index = 0; index != nbDevices; index++) {
        current_buttons = 0;
        device = m_devices.at(index);
        handle = m_handles.at(index);
        // begin haptics frame
        hHD = handle->m_device_handle;
        hdMakeCurrentDevice(hHD);
        hdBeginFrame(hHD);

        // get the current cartesian position of the device
        hdGetDoublev(HD_CURRENT_TRANSFORM, device->Frame4x4.Pointer());
        // retrieve cartesian velocities, write directly in state data
        hdGetDoublev(HD_CURRENT_VELOCITY,
                     device->m_measured_cv.VelocityLinear().Pointer());
        device->m_measured_cv.VelocityLinear().Multiply(cmn_mm);
        hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY,
                     device->m_measured_cv.VelocityAngular().Pointer());
        // retrieve forces
        hdGetDoublev(HD_LAST_FORCE,
                     device->m_measured_cf.Force().Pointer());

        // retrieve joint positions and efforts, write directly in state data
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES,
                     device->m_measured_js.Position().Pointer());
        hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES,
                     device->GimbalPositionJointRef.Pointer());
        hdGetDoublev(HD_LAST_JOINT_TORQUE,
                     device->m_measured_js.Effort().Pointer());
        hdGetDoublev(HD_LAST_GIMBAL_TORQUE,
                     device->GimbalEffortJointRef.Pointer());

        // retrieve the current button(s).
        hdGetIntegerv(HD_CURRENT_BUTTONS, &current_buttons);

        // apply forces
        hdSetDoublev(HD_CURRENT_FORCE, device->m_servo_cf.Force().Pointer());

        // end haptics frame
        hdEndFrame(hHD);

        // time stamp used to date data
        mtsStateIndex stateIndex = this->StateTable.GetIndexWriter();

        // copy transformation to the state table
        device->m_measured_cp.Position().Translation().Assign(device->Frame4x4TranslationRef);
        device->m_measured_cp.Position().Translation().Multiply(cmn_mm);
        device->m_measured_cp.Position().Rotation().Assign(device->Frame4x4RotationRef);

        // compare to previous value to create events
        if (current_buttons != device->m_buttons) {
            int currentButtonState, previousButtonState;
            prmEventButton event;
            event.Valid() = true;
            // test for button 1
            currentButtonState = current_buttons & HD_DEVICE_BUTTON_1;
            previousButtonState = device->m_buttons & HD_DEVICE_BUTTON_1;
            if (currentButtonState != previousButtonState) {
                if (currentButtonState == 0) {
                    event.SetType(prmEventButton::RELEASED);
                } else {
                    event.SetType(prmEventButton::PRESSED);
                }
                // throw the event
                device->m_button1_event(event);
            }
            // test for button 2
            currentButtonState = current_buttons & HD_DEVICE_BUTTON_2;
            previousButtonState = device->m_buttons & HD_DEVICE_BUTTON_2;
            if (currentButtonState != previousButtonState) {
                if (currentButtonState == 0) {
                    event.SetType(prmEventButton::RELEASED);
                } else {
                    event.SetType(prmEventButton::PRESSED);
                }
                // throw the event
                device->m_button2_event(event);
            }
            // save previous buttons state
            device->m_buttons = current_buttons;
        }

        // set all as valid
        device->m_measured_js.Valid() = true;
        device->m_measured_cp.Valid() = true;
        device->m_measured_cv.Valid() = true;
        device->m_measured_cf.Valid() = true;
    }

    // check for errors and abort the callback if a scheduler error
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        CMN_LOG_RUN_ERROR << "mtsSensableHDCallback: device error detected \""
                          << hdGetErrorString(error.errorCode) << "\"" << std::endl;
        // propagate error to all devices interfaces
        for (size_t index = 0; index != nbDevices; index++) {
            device = m_devices.at(index);
            // set all as invalid
            device->m_measured_js.Valid() = false;
            device->m_measured_cp.Valid() = false;
            device->m_measured_cv.Valid() = false;
            device->m_measured_cf.Valid() = false;
            device->m_interface->SendError(device->m_name + ": fatal error in scheduler callback \""
                                           + hdGetErrorString(error.errorCode) + "\"");
        }
        m_driver->CallbackReturnValue = HD_CALLBACK_DONE;
        return;
    }

    // send commands at the end
    ProcessQueuedCommands();

    // return flag to continue calling this function
    m_driver->CallbackReturnValue = HD_CALLBACK_CONTINUE;
}

void mtsSensableHD::Configure(const std::string & filename)
{
    // default case, no need to find device names in config file
    if (filename == "") {
        m_devices.resize(1);
        m_handles.resize(1);
        m_devices.at(0) = new mtsSensableHDDevice;
        m_devices.at(0)->m_name = "Default";
        SetupInterfaces();
        return;
    }

    // look for names in configuration file
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration\n"
                                 << jsonReader.getFormattedErrorMessages();
        exit(EXIT_FAILURE);
    }

    const Json::Value devices = jsonConfig["devices"];
    m_devices.resize(devices.size());
    m_handles.resize(devices.size());
    for (unsigned int index = 0; index < devices.size(); ++index) {
        m_devices.at(index) = new mtsSensableHDDevice;
        jsonValue = devices[index]["name"];
        if (!jsonValue.empty()) {
            m_devices.at(index)->m_name = jsonValue.asString();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: no \"name\" found for device " << index << std::endl;
            exit(EXIT_FAILURE);
        }
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found device \"" << m_devices.at(index)->m_name
                                   << "\"" << std::endl;
    }
    SetupInterfaces();
}

void mtsSensableHD::SetupInterfaces(void)
{
    m_driver = new mtsSensableHDDriverData;
    CMN_ASSERT(m_driver);

    const size_t nbDevices = m_devices.size();
    mtsSensableHDDevice * device;
    std::string interface_name;

    for (size_t index = 0; index != nbDevices; index++) {
        // use local data pointer to make code more readable
        device = m_devices.at(index);
        CMN_ASSERT(device);
        m_handles.at(index) = new mtsSensableHDHandle;

        device->m_operating_state.State() = prmOperatingState::DISABLED;

        device->Frame4x4TranslationRef.SetRef(device->Frame4x4.Column(3), 0);
        device->Frame4x4RotationRef.SetRef(device->Frame4x4, 0, 0);
        device->m_measured_js.Name().SetSize(NB_JOINTS);
        device->m_measured_js.Name().at(0) = "OuterYaw";
        device->m_measured_js.Name().at(1) = "OuterPitch1";
        device->m_measured_js.Name().at(2) = "OuterPitch2";
        device->m_measured_js.Name().at(3) = "Yaw";
        device->m_measured_js.Name().at(4) = "Pitch";
        device->m_measured_js.Name().at(5) = "Roll";
        device->m_measured_js.Position().SetSize(NB_JOINTS);
        device->GimbalPositionJointRef.SetRef(device->m_measured_js.Position(), 3, 3);
        device->m_measured_js.Effort().SetSize(NB_JOINTS);
        device->GimbalEffortJointRef.SetRef(device->m_measured_js.Effort(), 3, 3);

        device->m_configuration_js.Name() = device->m_measured_js.Name();
        device->m_configuration_js.Type().SetSize(NB_JOINTS);
        device->m_configuration_js.Type().SetAll(PRM_JOINT_REVOLUTE);
        device->m_configuration_js.Valid() = true;

        device->m_measured_cp.SetReferenceFrame(device->m_name + "_base");
        device->m_measured_cp.SetMovingFrame(device->m_name);
        // set to zero
        device->m_measured_js.Position().SetAll(0.0);
        device->m_measured_js.Effort().SetAll(0.0);
        device->m_measured_cv.VelocityLinear().SetAll(0.0);
        device->m_measured_cv.VelocityAngular().SetAll(0.0);
        device->m_measured_cf.Force().SetAll(0.0);

        // create interface with the device name, i.e. the map key
        CMN_LOG_CLASS_INIT_DEBUG << "SetupInterfaces: creating interface \""
                                 << device->m_name << "\"" << std::endl;
        mtsInterfaceProvided * provided = this->AddInterfaceProvided(device->m_name);
        device->m_interface = provided;

        // for Status, Warning and Error with mtsMessage
        provided->AddMessageEvents();

        // add the state data to the table
        this->StateTable.AddData(device->m_operating_state, device->m_name + "_operating_state");
        this->StateTable.AddData(device->m_measured_cp, device->m_name + "_measured_cp");
        this->StateTable.AddData(device->m_measured_cv, device->m_name + "_measured_cv");
        this->StateTable.AddData(device->m_measured_cf, device->m_name + "_measured_cf");
        this->StateTable.AddData(device->m_measured_js, device->m_name + "_measured_js");
        this->StateTable.AddData(device->m_servo_cf, device->m_name + "_servo_cf");
        this->StateTable.AddData(device->m_buttons, device->m_name + "_buttons");

        // provide read methods for state data
        provided->AddCommandReadState(this->StateTable, device->m_measured_cp,
                                      "measured_cp");
        provided->AddCommandReadState(this->StateTable, device->m_measured_cv,
                                      "measured_cv");
        provided->AddCommandReadState(this->StateTable, device->m_measured_cf,
                                      "measured_cf");
        provided->AddCommandReadState(this->StateTable, device->m_measured_js,
                                      "measured_js");

        // commands
        provided->AddCommandWrite(&mtsSensableHDDevice::servo_cf,
                                  device, "servo_cf");

        // add a method to read the current state index
        provided->AddCommandRead(&mtsStateTable::GetIndexReader, &StateTable,
                                 "GetStateIndex");

        // stats
        provided->AddCommandReadState(this->StateTable, StateTable.PeriodStats,
                                      "period_statistics");

        // robot state
        provided->AddCommandWrite(&mtsSensableHDDevice::state_command,
                                  device, "state_command");
        provided->AddCommandReadState(this->StateTable, device->m_operating_state,
                                      "operating_state");
        provided->AddEventWrite(device->m_operating_state_event, "operating_state",
                                prmOperatingState());

        // configuration
        this->mStateTableConfiguration.AddData(device->m_configuration_js,
                                               device->m_name + "ConfigurationJoint");
        this->AddStateTable(&mStateTableConfiguration);
        this->mStateTableConfiguration.SetAutomaticAdvance(false);
        this->mStateTableConfiguration.Start();
        this->mStateTableConfiguration.Advance();
        provided->AddCommandReadState(this->mStateTableConfiguration, device->m_configuration_js,
                                      "configuration_js");
        provided->AddCommandRead(&mtsSensableHDDevice::get_button_names,
                                 device, "get_button_names");

        // Add interfaces for button with events
        provided = this->AddInterfaceProvided(device->m_name + "Button1");
        provided->AddEventWrite(device->m_button1_event, "Button",
                                prmEventButton());
        provided = this->AddInterfaceProvided(device->m_name + "Button2");
        provided->AddEventWrite(device->m_button2_event, "Button",
                                prmEventButton());

        // This allows us to return Data->RetValue from the Run method.
        m_driver->CallbackReturnValue = HD_CALLBACK_CONTINUE;
        this->SetThreadReturnValue(static_cast<void *>(&m_driver->CallbackReturnValue));
    }
    CMN_LOG_CLASS_INIT_DEBUG << "SetupInterfaces: interfaces created: " << std::endl
                             << *this << std::endl;
}

mtsSensableHD::~mtsSensableHD()
{
    // free data object created using new
    if (m_driver) {
        delete m_driver;
        m_driver = 0;
    }
    const size_t nbDevices = m_devices.size();

    for (size_t index = 0; index != nbDevices; index++) {
        if (m_devices.at(index)) {
            delete m_devices.at(index);
            m_devices.at(index) = 0;
        }
        if (m_handles.at(index)) {
            delete m_handles.at(index);
            m_devices.at(index) = 0;
        }
    }
}

void mtsSensableHD::Create(void * CMN_UNUSED(data))
{
    const size_t nbDevices = m_devices.size();

    mtsSensableHDDevice * device;
    mtsSensableHDHandle * handle;
    HDErrorInfo error;

    CMN_ASSERT(m_driver);

    for (size_t index = 0; index != nbDevices; index++) {
        device = m_devices.at(index);
        handle = m_handles.at(index);
        CMN_ASSERT(device);
        handle->m_device_handle = hdInitDevice(device->m_name.c_str());
        if (HD_DEVICE_ERROR(error = hdGetError())) {
            device->m_interface->SendError(device->m_name + ": failed to initialize device \""
                                           + hdGetErrorString(error.errorCode) + "\"");
            device->m_device_available = false;
            return;
        } else {
            device->m_interface->SendStatus(device->m_name + ": device initialized");
        }
        device->m_device_available = true;
        device->m_interface->SendStatus(device->m_name + ": found device model \""
                                        + hdGetString(HD_DEVICE_MODEL_TYPE) + "\"");
        device->m_interface->SendStatus(device->m_name + ": found serial number \""
                                        + hdGetString(HD_DEVICE_SERIAL_NUMBER) + "\"");
        hdEnable(HD_FORCE_OUTPUT);
    }

    // Schedule the main callback that will communicate with the device
    m_driver->CallbackHandle = hdScheduleAsynchronous(mtsTaskFromCallbackAdapter::CallbackAdapter<HDCallbackCode>,
                                                      this->GetCallbackParameter(),
                                                      HD_MAX_SCHEDULER_PRIORITY);
    // Call base class Create function
    mtsTaskFromCallback::Create();

    // Set state to ready (cisstMultiTask component state, not robot state!)
    this->ChangeState(mtsComponentState::READY);
}

void mtsSensableHD::Start(void)
{
    HDErrorInfo error;
    hdSetSchedulerRate(1000);
    hdStartScheduler();
    // Check for errors
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        CMN_LOG_CLASS_INIT_ERROR << "Start: failed to start scheduler \""
                                 <<  hdGetErrorString(error.errorCode) << "\"" << std::endl;
        return;
    }
    // Call base class Start function
    mtsTaskFromCallback::Start();

    // Send status
    const size_t nbDevices = m_devices.size();
    mtsSensableHDDevice * device;
    for (size_t index = 0; index != nbDevices; index++) {
        device = m_devices.at(index);
        device->m_interface->SendStatus(device->m_name + ": scheduler started");
    }
}

void mtsSensableHD::Kill(void)
{
    // For cleanup, unschedule callback and stop the scheduler
    hdStopScheduler();
    hdUnschedule(m_driver->CallbackHandle);

    // Disable the devices
#if 0
    const size_t nbDevices = m_devices.size();
    mtsSensableHDDevice * device;
    mtsSensableHDHandle * handle;
    for (size_t index = 0; index != nbDevices; index++) {
        device = m_devices.at(index);
        handle = m_handles.at(index);
        if (device->m_device_available) {
            hdDisableDevice(handle->m_device_handle);
        }
    }
#endif

    // Call base class Kill function
    mtsTaskFromCallback::Kill();

    // Set state to finished
    this->ChangeState(mtsComponentState::FINISHED);
}

std::vector<std::string> mtsSensableHD::DeviceNames(void) const
{
    std::vector<std::string> result;
    const size_t nbDevices = m_devices.size();
    result.resize(nbDevices);
    mtsSensableHDDevice * device;
    for (size_t index = 0; index != nbDevices; index++) {
        device = m_devices.at(index);
        result.at(index) = device->m_name;
    }
    return result;
}
