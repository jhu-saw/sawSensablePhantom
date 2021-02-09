/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Anton Deguet
  Created on: 2008-04-04

  (C) Copyright 2008-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsSensableHD_h
#define _mtsSensableHD_h

#include <cisstMultiTask/mtsTaskFromCallback.h>

// Always include last
#include <sawSensablePhantom/sawSensablePhantomExport.h>

// forward declaration for private data
struct mtsSensableHDDriverData;
struct mtsSensableHDHandle;
class mtsSensableHDDevice;

class CISST_EXPORT mtsSensableHD: public mtsTaskFromCallbackAdapter {
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    /*! Default constructor, will use the default device connected and
      create and interface named "Default Arm" */
    inline mtsSensableHD(const std::string & componentName):
        mtsTaskFromCallbackAdapter(componentName, 5000),
        mStateTableConfiguration(100, "Configuration")
    {
    }

    inline mtsSensableHD(const mtsTaskConstructorArg & arg):
        mtsTaskFromCallbackAdapter(arg),
        mStateTableConfiguration(100, "Configuration")
    {
    }

    ~mtsSensableHD();
    void Configure(const std::string & filename = "");
	void Create(void * CMN_UNUSED(data) = 0);
    void Run(void);
	void Start(void);
	void Kill(void);
    void Cleanup(void) {};

    std::vector<std::string> DeviceNames(void) const;
    int DeviceCount;

 protected:

    mtsStateTable mStateTableConfiguration;
    typedef std::vector<mtsSensableHDDevice *> DevicesType;
    DevicesType m_devices;
    typedef std::vector<mtsSensableHDHandle *> HandlesType;
    HandlesType m_handles;
    mtsSensableHDDriverData * m_driver;
    void SetupInterfaces(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSensableHD);

#endif // _mtsSensableHD_h
