<!--ts-->
   * [sawSensablePhantom](#sawsensablephantom)
   * [Links](#links)
   * [Dependencies](#dependencies)
   * [Running the examples](#running-the-examples)
      * [Drivers](#drivers)
      * [Compilation](#compilation)
      * [Main example](#main-example)
      * [Calibration](#calibration)
      * [ROS](#ros)
         * [Sensable node](#sensable-node)
         * [RViz](#rviz)
         * [Python client](#python-client)
      * [OpenIGTLink (aka igtl)](#openigtlink-aka-igtl)
         * [Dynamic loading](#dynamic-loading)
         * [Testing with <em>pyigtl</em>](#testing-with-pyigtl)

<!-- Added by: anton, at: 2021-03-05T11:48-05:00 -->

<!--te-->

# sawSensablePhantom

This SAW component contains code for interfacing with the SensAble *PHANTOM Omni* and the Geomagic/3DS *Touch*.  It compiles on Windows and Linux.  It has been tested with:
  * Linux Ubuntu 16.04, 18.04 and 20.04 and Windows
  * SensAble *Phantom Omni* (FireWire/1394) and 3DS *Touch* (Ethernet) but it might work with other haptic devices from SensAble or 3DS (assuming the C API is the same)

The `ros` folder contains code for a ROS node that interfaces with the sawSensablePhantom component and publishes the 3D transformations, joint positions and efforts, buttons...  It also has subscribers to control the Omni (i.e. set cartesian wrench).  To build the ROS node, make sure you use `catkin build`.

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Sensable SDK and drivers, see below
 * Qt for user interface
 * ROS (optional)

# Running the examples

## Drivers

On windows, follow the instructions from the vendor.  On Linux, please read [drivers.md](drivers.md)

## Compilation

This code is part of the cisst-SAW libraries and components.  Once the drivers are installed, you can follow the *cisst-SAW* compilation instructions: https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake.

For Linux users, we strongly recommend to compile with ROS and the python catkin build tools (i.e. `catkin build`, NOT `catkin_make`).  Detailled instructions can be found on https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros.

Short version for Ubuntu (18.04) ROS (melodic) to compile using `catkin` and `wstool`:
```sh
sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev # most system dependencies we need
sudo apt install python-wstool python-catkin-tools # catkin and wstool for ROS build
source /opt/ros/melodic/setup.bash # or use whatever version of ROS is installed!
mkdir ~/catkin_ws # create the catkin workspace
cd ~/catkin_ws # go in the workspace
wstool init src # we're going to use wstool to pull all the code from github
catkin init # create files for catkin build tool
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # all code should be compiled in release mode
cd src # go in source directory to pull code
wstool merge https://github.com/jhu-saw/sawSensablePhantom/raw/devel/ros/sensable_phantom.rosinstall
wstool up # now wstool knows which repositories to pull, let's get the code
catkin build # ... and finally compile everything
```

## Main example

The main example provided is `sawSensablePhantomQtExample`.  The command line options are:
```sh
sawSensablePhantomQtExample:
 -j <value>, --json-config <value> : json configuration file (optional)
 -m, --component-manager : JSON files to configure component manager (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
```

To use the default Omni or only Omni on your computer:
```sh
sawSensablePhantomQtExample -D
```

If you want to use a specific Omni or define your own name for the device (e.g. Left vs Right):
```sh
sawSensablePhantomQtExample -D -j sawSensablePhantomLeft.json
```

Some examples of configuration files can be found in the `share` directory.  Here is an example:
```json
{
    "devices":
    [
        {
            "name": "left"
        }
    ]
}
```

## Calibration

We found that the Sensable *PHANTOM Omni* is not reporting the gimbal values the same way the Geomagic/3DS *Touch* does.  The issue might be due to lack of support for the calibration procedure in the older drivers/setup programs for the *PHANTOM Omni*.

So for this implementation we use the convention from the Geomagic/3DS *Touch*, i.e. the joint directions and origins are defined as follows:
* waist:
  * positive direction is toward left
  * zero is when facing user, grooves in house align between base and sphere
  * range is about -55 to 55
* shoulder:
  * positive direction is moving link up
  * zero is when link is horinzontal
  * range is about 0 to 100
* elbow:
  * positive direction is moving link up
  * zero is when link is orthogonal to previous link
  * range is about -45 to 70
* yaw:
  * positive direction is when rotating stylus to the right
  * zero is when stylus handle is towards user, grooves in housing align between wish-bone shaped link and arm
  * range is about -145 to 145
* pitch:
  * positive direction is when handle is lifted, tip going down
  * zero is when stylus is orthogonal to wish-bone link
  * range is about -80 to 55
* roll
  * position direction is when rolling to the right
  * zero is when buttons are up
  * range is about -150 to 150

We found that the older SensAble *PHANTOM Omni* didn't follow these convention for the last 3 joints (gimbal).  We haven't figured out an easy fix so we provide a way to load some scales and offsets in a custom configuration file that needs to be explicitely loaded:
```json
{
    "devices":
    [
        {
            "name": "Default PHANToM",  // name defined in driver/setup
            "rename": "arm",  // name from there on (optional), used for interfaces (GUI/ROS/IGTL)
            "servo_cf_viscosity": 3.0,
            "servo_cp_p_gain": 50.0,
            "servo_cp_d_gain": 5.0,
            "joint-scales": [1.0, 1.0, 1.0, 1.0, -1.0, 1.0],
            "joint-offsets": [0.0, 0.0, 0.0, 3.66519, -3.80482, 3.08923]
        }
    ]
}
```
To find the offsets, place the stylus in the inkwell then quit the application (we're not sure why but this is needed).  Then restart the application with stylus in the inkwell, stylus buttons on the top.  Check the reported joint angles in the GUI.  Ideally the values should be around [0, 15, -36, 0, 45, 0].  If they are not, compute the difference in degrees (**only for the last 3 values, leave the first 3 at zero**), convert to radians and update your device configuration file (see example above).  Then quit and restart the application to make sure your offsets are loaded properly.

As to why this is needed, our guess is that the first 3 joints are using relative encoders that can be preloaded when the stylus is in the inkwell.  The gimbal likely uses potentiometers (they're a bit noisy and always moving) so they could be calibrated just once but unfortunately we don't have access to that API so we have to maintain our own offsets.  Also, if you have a better solution, please let us know!

With ROS, start the application with a configuration file using:
```sh
roscd saw_sensable_phantom_config
rosrun sensable_phantom_ros sensable_phantom -j sawSensablePhantomDefault.json
```
Offsets might not be the same for all older *Omnis* so make sure the reported joint angles make sense.  You can also use ROS/RViz to visualize these angles (see below).

## ROS

### Sensable node

The ROS node is `sensable_phantom` and can be found in the package `sensable_phantom_ros`.  If you only have one Omni and don't need a configuration file:
```
rosrun sensable_phantom_ros sensable_phantom
```

If you have more than one Omni, please read the section above for the configuration file description.
```sh
roscd sensable_phantom_config
rosrun sensable_phantom_ros sensable_phantom -D -j sawSensablePhantomLeft.json
```

The ROS node has one more command line option:
```sh
 -j <value>, --json-config <value> : json configuration file (required)
 -p <value>, --ros-period <value> : period in seconds to read all tool positions (default 0.002, 2 ms, 500Hz).  There is no point to have a period higher than the device (optional)
 -m, --component-manager : JSON files to configure component manager (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
```

Once the node is started AND connected, the following ROS topics should appear:
```sh
/left/button1
/left/button2
/left/measured_cp
/left/measured_cv
/left/measured_js
/left/operating_state
/left/servo_cf
/left/servo_cp
/left/state_command
/stats/events/period_statistics
/stats/left/period_statistics
/stats/publishers_left/period_statistics
/stats/subscribers/period_statistics
```

### RViz

After you've started the `sensable_phantom` node, you can also visualize the Omni in RViz using:
```sh
roslaunch sensable_phantom_ros rviz.launch
```

If the arm has a different name (e.g. `right`), use:
```sh
roslaunch sensable_phantom_ros rviz.launch arm:=right
```

You can also start both the sensable node and RViz using:
```sh
roslaunch sensable_phantom_ros arm_rviz.launch
```

Note: urdf and models provided in this repository are based on files from https://github.com/fsuarez6/phantom_omni

### Python client

We use the [CRTK python client](https://github.com/collaborative-robotics/crtk_python_client) library to provide a simple Python API over ROS.  After building the code using `catkin build` and making sure your paths are set properly (something like `source ~/catkin_ws/devel/setup.bash`) you can use Python (or ipython) to do something like:
```python
import sensable
arm = sensable.arm('/arm') # the parameter is the ROS namespace for your device
arm.measured_cp() # returns a PyKDL frame with the current cartesian position
...
```

See script examples under `ros/scripts`.

## OpenIGTLink (aka `igtl`)

One can also communicate with the Omni using OpenIGTLink as
"middleware" instead of ROS.  To do so, you first need to make sure
sawOpenIGTLink is compiled against OpenIGTLink version 3+ (see https://github.com/jhu-saw/sawOpenIGTLink).  Once sawOpenIGTLink is
compiled, you will have two solutions.  You can either use the executable `sawSensablePhantomQtIGTL` which is linked against the *sawOpenIGTLink* library or use dynamic loading.

### Dynamic loading

you will need two extra configuration files (examples can be found in `share` directory):
* Component manager configuration file to load the dynamic library `sawOpenIGTLink` and create/configure the OpenIGTLink bridge.  The cisst component manager creates an instance of the class `mtsIGTLCRTKBridge`.  The component manager configuration file is passed to the main program using the option `-m`.
* IGTL bridge configuration file.  This indicates which component and interface to bridge to IGTL.  The component name is set in the `main()` function and should be `SensableHD`.  The interface name is the Omni name you've set when configuring your Omni (default is `Default PHANToM`).

### Testing with *pyigtl*

The first step is to install *pyigtl*.  See instructions on [*pyigtl* github page](https://github.com/lassoan/pyigtl).  You might have to use `pip3` instead of `pip`.  Then, start the `sawSensablePhantomQtIGTL` program.  In a different terminal, you can start `python` (or even better `ipython` or `ipython3`).  In Python:
```python
import pyigtl
client = pyigtl.OpenIGTLinkClient("127.0.0.1", 18944)
message = client.wait_for_message("arm/measured_cp") # get the current arm cartesian position
goal = message.matrix # copy the matrix part of the received message
transform_message = pyigtl.TransformMessage(goal, device_name="arm/servo_cp") # prepare a new message, to "servo" in cartesian position
client.send_message(transform_message) # send the message.  At that point, the Omni will try to stay in place
```

To disable position control, send a command to `servo_cf` with a null wrench (i.e. zero forces).
