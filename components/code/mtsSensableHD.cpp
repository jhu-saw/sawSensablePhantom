/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Anton Deguet
  Created on: 2008-04-04

  (C) Copyright 2008-2017 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmEventButton.h>

CMN_IMPLEMENT_SERVICES(mtsSensableHD);

struct mtsSensableHDDriverData {
    HDSchedulerHandle CallbackHandle;
    HDCallbackCode CallbackReturnValue;
};

struct mtsSensableHDHandle {
    HHD DeviceHandle;
};


// The task Run method
void mtsSensableHD::Run(void)
{
	ProcessQueuedCommands();

    int currentButtons;
    const size_t nbDevices = this->DevicesVector.size();
    DeviceData * deviceData;
    mtsSensableHDHandle * handle;
    HHD hHD;

    for (size_t index = 0; index != nbDevices; index++) {
        currentButtons = 0;
        deviceData = DevicesVector(index);
        handle = DevicesHandleVector(index);
        // begin haptics frame
        hHD = handle->DeviceHandle;
        hdMakeCurrentDevice(hHD);
        hdBeginFrame(hHD);

        // get the current cartesian position of the device
        hdGetDoublev(HD_CURRENT_TRANSFORM, deviceData->Frame4x4.Pointer());

        // retrieve cartesian velocities, write directly in state data
        hdGetDoublev(HD_CURRENT_VELOCITY, deviceData->VelocityCartesian.VelocityLinear().Pointer());
        hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, deviceData->VelocityCartesian.VelocityAngular().Pointer());

        // retrieve joint positions and efforts, write directly in state data
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, deviceData->StateJoint.Position().Pointer());
        hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, deviceData->GimbalPositionJointRef.Pointer());
        hdGetDoublev(HD_CURRENT_JOINT_TORQUE, deviceData->StateJoint.Effort().Pointer());
        hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, deviceData->GimbalEffortJointRef.Pointer());

        // retrieve the current button(s).
        hdGetIntegerv(HD_CURRENT_BUTTONS, &currentButtons);

        // apply forces
        if (deviceData->ForceOutputEnabled) {
            hdSetDoublev(HD_CURRENT_FORCE, deviceData->ForceCartesian.Force().Pointer());
        }
        // end haptics frame
        hdEndFrame(hHD);

        // time stamp used to date data
        mtsStateIndex stateIndex = this->StateTable.GetIndexWriter();

        // copy transformation to the state table
        deviceData->PositionCartesian.Position().Translation().Assign(deviceData->Frame4x4TranslationRef);
        deviceData->PositionCartesian.Position().Rotation().Assign(deviceData->Frame4x4RotationRef);

        // compare to previous value to create events
        if (currentButtons != deviceData->Buttons) {
            int currentButtonState, previousButtonState;
            prmEventButton event;
            // test for button 1
            currentButtonState = currentButtons & HD_DEVICE_BUTTON_1;
            previousButtonState = deviceData->Buttons & HD_DEVICE_BUTTON_1;
            if (currentButtonState != previousButtonState) {
                if (currentButtonState == 0) {
                    event.SetType(prmEventButton::RELEASED);
                } else {
                    event.SetType(prmEventButton::PRESSED);
                }
                // throw the event
                deviceData->Button1Event(event);
            }
            // test for button 2
            currentButtonState = currentButtons & HD_DEVICE_BUTTON_2;
            previousButtonState = deviceData->Buttons & HD_DEVICE_BUTTON_2;
            if (currentButtonState != previousButtonState) {
                if (currentButtonState == 0) {
                    deviceData->Clutch = false;
                    event.SetType(prmEventButton::RELEASED);
                } else {
                    deviceData->Clutch = true;
                    event.SetType(prmEventButton::PRESSED);
                }
                // throw the event
                deviceData->Button2Event(event);
            }
            // save previous buttons state
            deviceData->Buttons = currentButtons;
        }
    }

    // check for errors and abort the callback if a scheduler error
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        CMN_LOG_RUN_ERROR << "mtsSensableHDCallback: device error detected \""
                          << hdGetErrorString(error.errorCode) << "\"" << std::endl;
        // propagate error to all devices interfaces
        for (size_t index = 0; index != nbDevices; index++) {
            deviceData = DevicesVector(index);
            deviceData->Interface->SendError(deviceData->Name + ": fatal error in scheduler callback \""
                                             + hdGetErrorString(error.errorCode) + "\"");
        }
        this->Driver->CallbackReturnValue = HD_CALLBACK_DONE;
        return;
    }

    // return flag to continue calling this function
    this->Driver->CallbackReturnValue = HD_CALLBACK_CONTINUE;
}

mtsSensableHD::mtsSensableHD(const std::string & componentName):
    mtsTaskFromCallbackAdapter(componentName, 5000)
{
}

void mtsSensableHD::Configure(const std::string & filename)
{
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration\n"
                                 << jsonReader.getFormattedErrorMessages();
        return;
    }

    const Json::Value devices = jsonConfig["devices"];
    DevicesVector.SetSize(devices.size());
    DevicesHandleVector.SetSize(devices.size());
    for (unsigned int index = 0; index < devices.size(); ++index) {
        DevicesVector(index) = new DeviceData;
        DevicesVector(index)->DeviceNumber = index;
        jsonValue = devices[index]["name"];
        if (!jsonValue.empty()) {
            DevicesVector(index)->Name = jsonValue.asString();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: no \"name\" found for device " << index << std::endl;
        }
        jsonValue = devices[index]["force-enabled"];
        if (!jsonValue.empty()) {
            DevicesVector(index)->ForceOutputEnabled = jsonValue.asBool();
        } else {
            DevicesVector(index)->ForceOutputEnabled = true;
        }
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found device \"" << DevicesVector(index)->Name
                                   << "\" with force enabled set to \"" << DevicesVector(index)->ForceOutputEnabled
                                   << "\"" << std::endl;
    }

    SetupInterfaces();
}

void mtsSensableHD::SetupInterfaces(void)
{
    this->Driver = new mtsSensableHDDriverData;
    CMN_ASSERT(this->Driver);

    const size_t nbDevices = this->DevicesVector.size();
    DeviceData * deviceData;
    std::string interfaceName;
    mtsInterfaceProvided * providedInterface;

    for (size_t index = 0; index != nbDevices; index++) {
        // use local data pointer to make code more readable
        deviceData = DevicesVector(index);
        CMN_ASSERT(deviceData);
        interfaceName = DevicesVector(index)->Name;
        DevicesHandleVector(index) = new mtsSensableHDHandle;

        deviceData->Frame4x4TranslationRef.SetRef(deviceData->Frame4x4.Column(3), 0);
        deviceData->Frame4x4RotationRef.SetRef(deviceData->Frame4x4, 0, 0);
        deviceData->StateJoint.Name().SetSize(NB_JOINTS);
        deviceData->StateJoint.Name().at(0) = "OuterYaw";
        deviceData->StateJoint.Name().at(1) = "OuterPitch1";
        deviceData->StateJoint.Name().at(2) = "OuterPitch2";
        deviceData->StateJoint.Name().at(3) = "Yaw";
        deviceData->StateJoint.Name().at(4) = "Pitch";
        deviceData->StateJoint.Name().at(5) = "Roll";
        deviceData->StateJoint.Position().SetSize(NB_JOINTS);
        deviceData->GimbalPositionJointRef.SetRef(deviceData->StateJoint.Position(), 3, 3);
        deviceData->StateJoint.Effort().SetSize(NB_JOINTS);
        deviceData->GimbalEffortJointRef.SetRef(deviceData->StateJoint.Effort(), 3, 3);

        // set to zero
        deviceData->StateJoint.Position().SetAll(0.0);
        deviceData->StateJoint.Effort().SetAll(0.0);
        deviceData->VelocityCartesian.VelocityLinear().SetAll(0.0);
        deviceData->VelocityCartesian.VelocityAngular().SetAll(0.0);
        deviceData->Clutch = false;

        // create interface with the device name, i.e. the map key
        CMN_LOG_CLASS_INIT_DEBUG << "SetupInterfaces: creating interface \"" << interfaceName << "\"" << std::endl;
        deviceData->Interface = this->AddInterfaceProvided(interfaceName);

        // for Status, Warning and Error with mtsMessage
        deviceData->Interface->AddMessageEvents();

        // Stats
        deviceData->Interface->AddCommandReadState(this->StateTable, StateTable.PeriodStats,
                                                   "GetPeriodStatistics");

        // add the state data to the table
        this->StateTable.AddData(deviceData->PositionCartesian, interfaceName + "PositionCartesian");
        this->StateTable.AddData(deviceData->VelocityCartesian, interfaceName + "VelocityCartesian");
        this->StateTable.AddData(deviceData->StateJoint, interfaceName + "StateJoint");
        this->StateTable.AddData(deviceData->Buttons, interfaceName + "Buttons");

        if (deviceData->ForceOutputEnabled) {
            this->StateTable.AddData(deviceData->ForceCartesian, interfaceName + "ForceCartesian");
            deviceData->Interface->AddCommandWriteState(this->StateTable,
                                                        deviceData->ForceCartesian,
                                                        "SetForceCartesian");
        }

        // provide read methods for state data
        deviceData->Interface->AddCommandReadState(this->StateTable,
                                               deviceData->PositionCartesian,
                                               "GetPositionCartesian");
        deviceData->Interface->AddCommandReadState(this->StateTable,
                                               deviceData->VelocityCartesian,
                                               "GetVelocityCartesian");
        deviceData->Interface->AddCommandReadState(this->StateTable,
                                               deviceData->StateJoint,
                                               "GetStateJoint");

        // add a method to read the current state index
        deviceData->Interface->AddCommandRead(&mtsStateTable::GetIndexReader, &StateTable,
                                          "GetStateIndex");

        // Add interfaces for button with events
        providedInterface = this->AddInterfaceProvided(interfaceName + "Button1");
        providedInterface->AddEventWrite(deviceData->Button1Event, "Button",
                                         prmEventButton());
        providedInterface = this->AddInterfaceProvided(interfaceName + "Button2");
        providedInterface->AddEventWrite(deviceData->Button2Event, "Button",
                                         prmEventButton());

        // This allows us to return Data->RetValue from the Run method.
        this->Driver->CallbackReturnValue = HD_CALLBACK_CONTINUE;
        this->SetThreadReturnValue(static_cast<void *>(&this->Driver->CallbackReturnValue));
    }
    CMN_LOG_CLASS_INIT_DEBUG << "SetupInterfaces: interfaces created: " << std::endl
                             << *this << std::endl;
}

mtsSensableHD::~mtsSensableHD()
{
    // free data object created using new
    if (this->Driver) {
        delete this->Driver;
        this->Driver = 0;
    }
    const size_t nbDevices = this->DevicesVector.size();

    for (size_t index = 0; index != nbDevices; index++) {
        if (DevicesVector(index)) {
            delete DevicesVector(index);
            DevicesVector(index) = 0;
        }
        if (DevicesHandleVector(index)) {
            delete DevicesHandleVector(index);
            DevicesHandleVector(index) = 0;
        }
    }
}

void mtsSensableHD::Create(void * CMN_UNUSED(data))
{
    const size_t nbDevices = this->DevicesVector.size();

    DeviceData * deviceData;
    mtsSensableHDHandle * handle;
    std::string interfaceName;
    HDErrorInfo error;

    CMN_ASSERT(this->Driver);

    for (size_t index = 0; index != nbDevices; index++) {
        deviceData = DevicesVector(index);
        interfaceName = DevicesVector(index)->Name;
        handle = DevicesHandleVector(index);
        CMN_ASSERT(deviceData);
        handle->DeviceHandle = hdInitDevice(interfaceName.c_str());
        if (HD_DEVICE_ERROR(error = hdGetError())) {
            deviceData->Interface->SendError(deviceData->Name + ": failed to initialize device \""
                                             + hdGetErrorString(error.errorCode) + "\"");
            deviceData->DeviceEnabled = false;
            return;
        } else {
            deviceData->Interface->SendStatus(deviceData->Name + ": device initialized");
        }
        deviceData->DeviceEnabled = true;
        deviceData->Interface->SendStatus(deviceData->Name + ": found device model \""
                                          + hdGetString(HD_DEVICE_MODEL_TYPE) + "\"");
        if (deviceData->ForceOutputEnabled) {
            hdEnable(HD_FORCE_OUTPUT);
        } else {
            hdDisable(HD_FORCE_OUTPUT);
        }
    }

    // Schedule the main callback that will communicate with the device
    this->Driver->CallbackHandle = hdScheduleAsynchronous(mtsTaskFromCallbackAdapter::CallbackAdapter<HDCallbackCode>,
                                                          this->GetCallbackParameter(),
                                                          HD_MAX_SCHEDULER_PRIORITY);
    // Call base class Create function
    mtsTaskFromCallback::Create();

    // Set state to ready
    this->ChangeState(mtsComponentState::READY);
}

void mtsSensableHD::Start(void)
{
    HDErrorInfo error;
    hdSetSchedulerRate(500);
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
    const size_t nbDevices = this->DevicesVector.size();
    DeviceData * deviceData;
    for (size_t index = 0; index != nbDevices; index++) {
        deviceData = DevicesVector(index);
        deviceData->Interface->SendStatus(deviceData->Name + ": scheduler started");
    }
}

void mtsSensableHD::Kill(void)
{
    // For cleanup, unschedule callback and stop the scheduler
    hdStopScheduler();
    hdUnschedule(this->Driver->CallbackHandle);

    // Disable the devices
    const size_t nbDevices = this->DevicesVector.size();
    DeviceData * deviceData;
    mtsSensableHDHandle * handle;
    for (size_t index = 0; index != nbDevices; index++) {
        deviceData = DevicesVector(index);
        handle = DevicesHandleVector(index);
        if (deviceData->DeviceEnabled) {
            hdDisableDevice(handle->DeviceHandle);
        }
    }

    // Call base class Kill function
    mtsTaskFromCallback::Kill();

    // Set state to finished
    this->ChangeState(mtsComponentState::FINISHED);
}

std::vector<std::string> mtsSensableHD::DeviceNames(void) const
{
    std::vector<std::string> result;
    const size_t nbDevices = this->DevicesVector.size();
    result.resize(nbDevices);
    DeviceData * deviceData;
    for (size_t index = 0; index != nbDevices; index++) {
        deviceData = DevicesVector(index);
        result.at(index) = deviceData->Name;
    }
    return result;
}
