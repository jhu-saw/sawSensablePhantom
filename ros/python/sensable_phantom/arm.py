#  Author(s):  Anton Deguet
#  Created on: 2021-03-04

# (C) Copyright 2012-2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk
import PyKDL

class arm(object):
    """Simple arm API wrapping around ROS messages
    """

    # initialize the arm
    def __init__(self, ral, arm_name, expected_interval = 0.01):
        """Constructor.  This initializes a few data members.It
        requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `arm`, the ROS topics will be from the namespace
        `arm`"""
        # ros stuff
        self.__ral = ral.create_child(arm_name)
        self.__arm_name = arm_name

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_measured_cv()
        self.__crtk_utils.add_servo_cp()
        self.__crtk_utils.add_servo_cf()

    def ral(self):
        return self.__ral

    def name(self):
        return self.__arm_name
