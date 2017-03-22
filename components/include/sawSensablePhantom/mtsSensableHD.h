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

#ifndef _mtsSensableHD_h
#define _mtsSensableHD_h

#include <cisstMultiTask/mtsTaskFromCallback.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

// Always include last
#include <sawSensablePhantom/sawSensablePhantomExport.h>

// forward declaration for private data
struct mtsSensableHDDriverData;
struct mtsSensableHDHandle;

class CISST_EXPORT mtsSensableHD: public mtsTaskFromCallbackAdapter {
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    enum {NB_JOINTS = 6};
    int DeviceCount;

protected:
    // internal data using Sensable data types
    struct DeviceData {
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

    vctDynamicVector<DeviceData *> DevicesVector;
    vctDynamicVector<mtsSensableHDHandle *> DevicesHandleVector;
    mtsSensableHDDriverData * Driver;
    void SetupInterfaces(void);

public:
    /*! Default constructor, will use the default device connected and
      create and interface named "Default Arm" */
    mtsSensableHD(const std::string & componentName);

    ~mtsSensableHD();
    void Configure(const std::string & filename = "");
	void Create(void * CMN_UNUSED(data) = 0);
    void Run(void);
	void Start(void);
	void Kill(void);
    void Cleanup(void) {};

    std::vector<std::string> DeviceNames(void) const;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSensableHD);

#endif // _mtsSensableHD_h
