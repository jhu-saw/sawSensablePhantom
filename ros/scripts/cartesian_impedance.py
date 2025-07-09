#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2017-07-22

# (C) Copyright 2017-2022 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun sensable_phantom_ros sensable_phantom

# To communicate with the arm using ROS topics, see the python based example arm_arm_test.py:
# > rosrun sensable_phantom_ros cartesian_impedance.py -a arm

import sensable
import sys
import rospy
import numpy
import threading
import argparse
from sensor_msgs.msg import Joy
from cisst_msgs.msg import prmCartesianImpedanceGains

# print with node id
def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, expected_interval):
        print_id('configuring cartesian_impedance for %s' % robot_name)
        self.expected_interval = expected_interval
        self.arm = sensable.arm(arm_name = robot_name,
                                expected_interval = expected_interval)
        self.button_event = threading.Event()
        rospy.Subscriber(self.arm._arm__full_ros_namespace + '/button1',
                         Joy, self.button_event_cb)
        self.set_gains_pub = rospy.Publisher(self.arm._arm__full_ros_namespace + '/servo_ci',
                                             prmCartesianImpedanceGains, latch = True, queue_size = 1)

    # homing example
    def home(self):
        print_id('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print_id('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')

    # foot pedal callback
    def button_event_cb(self, data):
        if (data.buttons[0] == 1):
            self.button_event.set()

    # wait for foot pedal
    def wait_for_button(self):
        self.button_event.clear()
        self.button_event.wait(600)


    # tests
    def tests(self):
        gains = prmCartesianImpedanceGains()
        # set orientation to identity quaternions
        gains.ForceOrientation.w = 1.0
        gains.TorqueOrientation.w = 1.0

        print_id('press button 1 to move to the next example')

        print_id('arm will be constrained in X/Z plane around the current position')
        self.wait_for_button()
        # set gains in z direction
        gains.PosStiffNeg.y = -200.0
        gains.PosStiffPos.y = -200.0
        gains.PosDampingNeg.y = -5.0
        gains.PosDampingPos.y = -5.0
        gains.ForcePosition.x = self.arm.measured_cp().p[0]
        gains.ForcePosition.y = self.arm.measured_cp().p[1]
        gains.ForcePosition.z = self.arm.measured_cp().p[2]
        self.set_gains_pub.publish(gains)

        print_id('arm will be constrained in X/Z half plane around the current position')
        self.wait_for_button()
        # set gains in y direction, stiffer in half positive, 0 in negative
        gains.PosStiffNeg.y = -200.0
        gains.PosStiffPos.y = 0.0
        gains.PosDampingNeg.y = -5.0
        gains.PosDampingPos.y = 0.0
        gains.ForcePosition.x = self.arm.measured_cp().p[0]
        gains.ForcePosition.y = self.arm.measured_cp().p[1]
        gains.ForcePosition.z = self.arm.measured_cp().p[2]
        self.set_gains_pub.publish(gains)

        print_id('a vertical line will be created around the current position, with viscosity along the line')
        self.wait_for_button()
        # set gains in x, z directions for the line
        gains.PosStiffNeg.x = -200.0
        gains.PosStiffPos.x = -200.0
        gains.PosDampingNeg.x = -5.0
        gains.PosDampingPos.x = -5.0
        gains.PosStiffNeg.z = -200.0
        gains.PosStiffPos.z = -200.0
        gains.PosDampingNeg.z = -5.0
        gains.PosDampingPos.z = -5.0
        # viscosity along the line
        gains.PosStiffNeg.y = 0.0
        gains.PosStiffPos.y = 0.0
        gains.PosDampingNeg.y = -5.0
        gains.PosDampingPos.y = -5.0
        # always start from current position to avoid jumps
        gains.ForcePosition.x = self.arm.measured_cp().p[0]
        gains.ForcePosition.y = self.arm.measured_cp().p[1]
        gains.ForcePosition.z = self.arm.measured_cp().p[2]
        self.set_gains_pub.publish(gains)

        print_id('a plane will be created perpendicular to the master gripper')
        self.wait_for_button()
        # set gains in x, z directions for the line
        gains.PosStiffNeg.x = 0.0
        gains.PosStiffPos.x = 0.0
        gains.PosDampingNeg.x = 0.0
        gains.PosDampingPos.x = 0.0
        gains.PosStiffNeg.y = 0.0
        gains.PosStiffPos.y = 0.0
        gains.PosDampingNeg.y = 0.0
        gains.PosDampingPos.y = 0.0
        gains.PosStiffNeg.z = -200.0
        gains.PosStiffPos.z = -200.0
        gains.PosDampingNeg.z = -5.0
        gains.PosDampingPos.z = -5.0

        stiffOri = -0.2
        dampOri = -0.01
        gains.OriStiffNeg.x = stiffOri
        gains.OriStiffPos.x = stiffOri
        gains.OriDampingNeg.x = dampOri
        gains.OriDampingPos.x = dampOri
        gains.OriStiffNeg.y = stiffOri
        gains.OriStiffPos.y = stiffOri
        gains.OriDampingNeg.y = dampOri
        gains.OriDampingPos.y = dampOri
        gains.OriStiffNeg.z = 0.0
        gains.OriStiffPos.z = 0.0
        gains.OriDampingNeg.z = 0.0
        gains.OriDampingPos.z = 0.0

        # always start from current position to avoid jumps
        gains.ForcePosition.x = self.arm.measured_cp().p[0]
        gains.ForcePosition.y = self.arm.measured_cp().p[1]
        gains.ForcePosition.z = self.arm.measured_cp().p[2]
        orientationQuaternion = self.arm.measured_cp().M.GetQuaternion()
        gains.ForceOrientation.x = orientationQuaternion[0]
        gains.ForceOrientation.y = orientationQuaternion[1]
        gains.ForceOrientation.z = orientationQuaternion[2]
        gains.ForceOrientation.w = orientationQuaternion[3]
        gains.TorqueOrientation.x = orientationQuaternion[0]
        gains.TorqueOrientation.y = orientationQuaternion[1]
        gains.TorqueOrientation.z = orientationQuaternion[2]
        gains.TorqueOrientation.w = orientationQuaternion[3]
        self.set_gains_pub.publish(gains)

        print_id('press button 1 to end')
        self.wait_for_button()

        # release the arm by sending zero wrench
        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm.servo_cf(wrench)

    # main method
    def run(self):
        self.home()
        self.tests()

if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('cartesian_impedance')
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = example_application()
    application.configure(args.arm, args.interval)
    application.run()
