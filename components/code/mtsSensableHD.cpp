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

#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmEventButton.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsSensableHD, mtsTaskFromCallback, mtsTaskConstructorArg);

struct mtsSensableHDDriverData {
    HDSchedulerHandle CallbackHandle;
    HDCallbackCode CallbackReturnValue;
};

struct mtsSensableHDHandle {
    HHD DeviceHandle;
};

// internal data using Sensable data types
class mtsSensableHDDevice {
public:
    inline void SetDesiredState(const std::string & state) {
    }
    inline void GetDesiredState(std::string & state) const {
    }
    inline void GetCurrentState(std::string & state) const {
    }
    void SetPositionCartesian(const prmForceCartesianSet & desiredForceTorque);
    void SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition);
    void SetWrenchBody(const prmForceCartesianSet & newForce);
    void SetGravityCompensation(const bool & gravityCompensation);
    void LockOrientation(const vctMatRot3 & orientation);
    void UnlockOrientation(void);
    void Freeze(void);

    mtsInterfaceProvided * Interface;

    bool DeviceEnabled;
    bool ForceOutputEnabled;

    // local copy of the buttons state as defined by Sensable
    int Buttons;

    // local copy of the position and velocities
    prmPositionCartesianGet PositionCartesian;
    prmVelocityCartesianGet VelocityCartesian;
    prmStateJoint StateJoint;
    vctDynamicVectorRef<double> GimbalPositionJointRef, GimbalEffortJointRef;

    // mtsFunction called to broadcast the event
    mtsFunctionWrite Button1Event, Button2Event;

    prmForceCartesianSet ForceCartesian;

    // local buffer used to store the position as provided
    // by Sensable
    typedef vctFixedSizeMatrix<double, 4, 4, VCT_COL_MAJOR> Frame4x4Type;
    Frame4x4Type Frame4x4;
    vctFixedSizeConstVectorRef<double, 3, Frame4x4Type::ROWSTRIDE> Frame4x4TranslationRef;
    vctFixedSizeConstMatrixRef<double, 3, 3,
                               Frame4x4Type::ROWSTRIDE, Frame4x4Type::COLSTRIDE> Frame4x4RotationRef;

    bool Clutch;
    std::string Name;
    int DeviceNumber;
};


// The task Run method
void mtsSensableHD::Run(void)
{
    int currentButtons;
    const size_t nbDevices = mDevices.size();
    mtsSensableHDDevice * device;
    mtsSensableHDHandle * handle;
    HHD hHD;

    for (size_t index = 0; index != nbDevices; index++) {
        currentButtons = 0;
        device = mDevices.at(index);
        handle = mHandles.at(index);
        // begin haptics frame
        hHD = handle->DeviceHandle;
        hdMakeCurrentDevice(hHD);
        hdBeginFrame(hHD);

        // get the current cartesian position of the device
        hdGetDoublev(HD_CURRENT_TRANSFORM, device->Frame4x4.Pointer());

        // retrieve cartesian velocities, write directly in state data
        hdGetDoublev(HD_CURRENT_VELOCITY, device->VelocityCartesian.VelocityLinear().Pointer());
        hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, device->VelocityCartesian.VelocityAngular().Pointer());

        // retrieve joint positions and efforts, write directly in state data
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, device->StateJoint.Position().Pointer());
        hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, device->GimbalPositionJointRef.Pointer());
        hdGetDoublev(HD_CURRENT_JOINT_TORQUE, device->StateJoint.Effort().Pointer());
        hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, device->GimbalEffortJointRef.Pointer());

        // retrieve the current button(s).
        hdGetIntegerv(HD_CURRENT_BUTTONS, &currentButtons);

        // apply forces
        if (device->ForceOutputEnabled) {
            hdSetDoublev(HD_CURRENT_FORCE, device->ForceCartesian.Force().Pointer());
        }
        // end haptics frame
        hdEndFrame(hHD);

        // time stamp used to date data
        mtsStateIndex stateIndex = this->StateTable.GetIndexWriter();

        // copy transformation to the state table
        device->PositionCartesian.Position().Translation().Assign(device->Frame4x4TranslationRef);
        device->PositionCartesian.Position().Translation().Multiply(cmn_mm);
        device->PositionCartesian.Position().Rotation().Assign(device->Frame4x4RotationRef);

        // compare to previous value to create events
        if (currentButtons != device->Buttons) {
            int currentButtonState, previousButtonState;
            prmEventButton event;
            // test for button 1
            currentButtonState = currentButtons & HD_DEVICE_BUTTON_1;
            previousButtonState = device->Buttons & HD_DEVICE_BUTTON_1;
            if (currentButtonState != previousButtonState) {
                if (currentButtonState == 0) {
                    event.SetType(prmEventButton::RELEASED);
                } else {
                    event.SetType(prmEventButton::PRESSED);
                }
                // throw the event
                device->Button1Event(event);
            }
            // test for button 2
            currentButtonState = currentButtons & HD_DEVICE_BUTTON_2;
            previousButtonState = device->Buttons & HD_DEVICE_BUTTON_2;
            if (currentButtonState != previousButtonState) {
                if (currentButtonState == 0) {
                    device->Clutch = false;
                    event.SetType(prmEventButton::RELEASED);
                } else {
                    device->Clutch = true;
                    event.SetType(prmEventButton::PRESSED);
                }
                // throw the event
                device->Button2Event(event);
            }
            // save previous buttons state
            device->Buttons = currentButtons;
        }

        // set all as valid
        device->StateJoint.Valid() = true;
        device->PositionCartesian.Valid() = true;
        device->VelocityCartesian.Valid() = true;
    }

    // check for errors and abort the callback if a scheduler error
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        CMN_LOG_RUN_ERROR << "mtsSensableHDCallback: device error detected \""
                          << hdGetErrorString(error.errorCode) << "\"" << std::endl;
        // propagate error to all devices interfaces
        for (size_t index = 0; index != nbDevices; index++) {
            device = mDevices.at(index);
            // set all as invalid
            device->StateJoint.Valid() = false;
            device->PositionCartesian.Valid() = false;
            device->VelocityCartesian.Valid() = false;
            device->Interface->SendError(device->Name + ": fatal error in scheduler callback \""
                                         + hdGetErrorString(error.errorCode) + "\"");
        }
        mDriver->CallbackReturnValue = HD_CALLBACK_DONE;
        return;
    }

    // send commands at the end
    ProcessQueuedCommands();

    // return flag to continue calling this function
    mDriver->CallbackReturnValue = HD_CALLBACK_CONTINUE;
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
    mDevices.resize(devices.size());
    mHandles.resize(devices.size());
    for (unsigned int index = 0; index < devices.size(); ++index) {
        mDevices.at(index) = new mtsSensableHDDevice;
        mDevices.at(index)->DeviceNumber = index;
        jsonValue = devices[index]["name"];
        if (!jsonValue.empty()) {
            mDevices.at(index)->Name = jsonValue.asString();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: no \"name\" found for device " << index << std::endl;
        }
        jsonValue = devices[index]["force-enabled"];
        if (!jsonValue.empty()) {
            mDevices.at(index)->ForceOutputEnabled = jsonValue.asBool();
        } else {
            mDevices.at(index)->ForceOutputEnabled = true;
        }
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found device \"" << mDevices.at(index)->Name
                                   << "\" with force enabled set to \"" << mDevices.at(index)->ForceOutputEnabled
                                   << "\"" << std::endl;
    }

    SetupInterfaces();
}

void mtsSensableHD::SetupInterfaces(void)
{
    mDriver = new mtsSensableHDDriverData;
    CMN_ASSERT(mDriver);

    const size_t nbDevices = mDevices.size();
    mtsSensableHDDevice * device;
    std::string interfaceName;
    mtsInterfaceProvided * providedInterface;

    for (size_t index = 0; index != nbDevices; index++) {
        // use local data pointer to make code more readable
        device = mDevices.at(index);
        CMN_ASSERT(device);
        interfaceName = device->Name;
        mHandles.at(index) = new mtsSensableHDHandle;

        device->Frame4x4TranslationRef.SetRef(device->Frame4x4.Column(3), 0);
        device->Frame4x4RotationRef.SetRef(device->Frame4x4, 0, 0);
        device->StateJoint.Name().SetSize(NB_JOINTS);
        device->StateJoint.Name().at(0) = "OuterYaw";
        device->StateJoint.Name().at(1) = "OuterPitch1";
        device->StateJoint.Name().at(2) = "OuterPitch2";
        device->StateJoint.Name().at(3) = "Yaw";
        device->StateJoint.Name().at(4) = "Pitch";
        device->StateJoint.Name().at(5) = "Roll";
        device->StateJoint.Type().SetSize(NB_JOINTS);
        device->StateJoint.Type().SetAll(PRM_JOINT_REVOLUTE);
        device->StateJoint.Position().SetSize(NB_JOINTS);
        device->GimbalPositionJointRef.SetRef(device->StateJoint.Position(), 3, 3);
        device->StateJoint.Effort().SetSize(NB_JOINTS);
        device->GimbalEffortJointRef.SetRef(device->StateJoint.Effort(), 3, 3);

        device->PositionCartesian.Valid() = false;
        device->PositionCartesian.SetReferenceFrame(device->Name + "_base");
        device->PositionCartesian.SetMovingFrame(device->Name);
        // set to zero
        device->StateJoint.Position().SetAll(0.0);
        device->StateJoint.Effort().SetAll(0.0);
        device->VelocityCartesian.VelocityLinear().SetAll(0.0);
        device->VelocityCartesian.VelocityAngular().SetAll(0.0);
        device->Clutch = false;

        // create interface with the device name, i.e. the map key
        CMN_LOG_CLASS_INIT_DEBUG << "SetupInterfaces: creating interface \"" << interfaceName << "\"" << std::endl;
        device->Interface = this->AddInterfaceProvided(interfaceName);

        // for Status, Warning and Error with mtsMessage
        device->Interface->AddMessageEvents();

        // Stats
        device->Interface->AddCommandReadState(this->StateTable, StateTable.PeriodStats,
                                               "GetPeriodStatistics");

        // robot State
        device->Interface->AddCommandWrite(&mtsSensableHDDevice::SetDesiredState,
                                           device, "SetDesiredState", std::string(""));
        device->Interface->AddCommandRead(&mtsSensableHDDevice::GetDesiredState,
                                          device, "GetDesiredState", std::string(""));
        device->Interface->AddCommandRead(&mtsSensableHDDevice::GetCurrentState,
                                          device, "GetCurrentState", std::string(""));

        // add the state data to the table
        this->StateTable.AddData(device->PositionCartesian, interfaceName + "PositionCartesian");
        this->StateTable.AddData(device->VelocityCartesian, interfaceName + "VelocityCartesian");
        this->StateTable.AddData(device->StateJoint, interfaceName + "StateJoint");
        this->StateTable.AddData(device->Buttons, interfaceName + "Buttons");

        if (device->ForceOutputEnabled) {
            this->StateTable.AddData(device->ForceCartesian, interfaceName + "ForceCartesian");
            device->Interface->AddCommandWriteState(this->StateTable,
                                                    device->ForceCartesian,
                                                    "SetWrenchBody");
        }

        // provide read methods for state data
        device->Interface->AddCommandReadState(this->StateTable,
                                               device->PositionCartesian,
                                               "GetPositionCartesian");
        device->Interface->AddCommandReadState(this->StateTable,
                                               device->VelocityCartesian,
                                               "GetVelocityCartesian");
        device->Interface->AddCommandReadState(this->StateTable,
                                               device->StateJoint,
                                               "GetStateJoint");

        // add a method to read the current state index
        device->Interface->AddCommandRead(&mtsStateTable::GetIndexReader, &StateTable,
                                          "GetStateIndex");

        // Add interfaces for button with events
        providedInterface = this->AddInterfaceProvided(interfaceName + "Button1");
        providedInterface->AddEventWrite(device->Button1Event, "Button",
                                         prmEventButton());
        providedInterface = this->AddInterfaceProvided(interfaceName + "Button2");
        providedInterface->AddEventWrite(device->Button2Event, "Button",
                                         prmEventButton());

        // This allows us to return Data->RetValue from the Run method.
        mDriver->CallbackReturnValue = HD_CALLBACK_CONTINUE;
        this->SetThreadReturnValue(static_cast<void *>(&mDriver->CallbackReturnValue));
    }
    CMN_LOG_CLASS_INIT_DEBUG << "SetupInterfaces: interfaces created: " << std::endl
                             << *this << std::endl;
}

mtsSensableHD::~mtsSensableHD()
{
    // free data object created using new
    if (mDriver) {
        delete mDriver;
        mDriver = 0;
    }
    const size_t nbDevices = mDevices.size();

    for (size_t index = 0; index != nbDevices; index++) {
        if (mDevices.at(index)) {
            delete mDevices.at(index);
            mDevices.at(index) = 0;
        }
        if (mHandles.at(index)) {
            delete mHandles.at(index);
            mDevices.at(index) = 0;
        }
    }
}

void mtsSensableHD::Create(void * CMN_UNUSED(data))
{
    const size_t nbDevices = mDevices.size();

    mtsSensableHDDevice * device;
    mtsSensableHDHandle * handle;
    std::string interfaceName;
    HDErrorInfo error;

    CMN_ASSERT(mDriver);

    for (size_t index = 0; index != nbDevices; index++) {
        device = mDevices.at(index);
        interfaceName = device->Name;
        handle = mHandles.at(index);
        CMN_ASSERT(device);
        handle->DeviceHandle = hdInitDevice(interfaceName.c_str());
        if (HD_DEVICE_ERROR(error = hdGetError())) {
            device->Interface->SendError(device->Name + ": failed to initialize device \""
                                         + hdGetErrorString(error.errorCode) + "\"");
            device->DeviceEnabled = false;
            return;
        } else {
            device->Interface->SendStatus(device->Name + ": device initialized");
        }
        device->DeviceEnabled = true;
        device->Interface->SendStatus(device->Name + ": found device model \""
                                      + hdGetString(HD_DEVICE_MODEL_TYPE) + "\"");
        if (device->ForceOutputEnabled) {
            hdEnable(HD_FORCE_OUTPUT);
        } else {
            hdDisable(HD_FORCE_OUTPUT);
        }
    }

    // Schedule the main callback that will communicate with the device
    mDriver->CallbackHandle = hdScheduleAsynchronous(mtsTaskFromCallbackAdapter::CallbackAdapter<HDCallbackCode>,
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
    const size_t nbDevices = mDevices.size();
    mtsSensableHDDevice * device;
    for (size_t index = 0; index != nbDevices; index++) {
        device = mDevices.at(index);
        device->Interface->SendStatus(device->Name + ": scheduler started");
    }
}

void mtsSensableHD::Kill(void)
{
    // For cleanup, unschedule callback and stop the scheduler
    hdStopScheduler();
    hdUnschedule(mDriver->CallbackHandle);

    // Disable the devices
    const size_t nbDevices = mDevices.size();
    mtsSensableHDDevice * device;
    mtsSensableHDHandle * handle;
    for (size_t index = 0; index != nbDevices; index++) {
        device = mDevices.at(index);
        handle = mHandles.at(index);
        if (device->DeviceEnabled) {
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
    const size_t nbDevices = mDevices.size();
    result.resize(nbDevices);
    mtsSensableHDDevice * device;
    for (size_t index = 0; index != nbDevices; index++) {
        device = mDevices.at(index);
        result.at(index) = device->Name;
    }
    return result;
}
