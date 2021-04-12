<!--ts-->
   * [sawSensablePhantom](#sawsensablephantom)
   * [Links](#links)
   * [Dependencies](#dependencies)
   * [Running the examples](#running-the-examples)
      * [Linux installation](#linux-installation)
      * [Configuration](#configuration)
      * [Compilation](#compilation)
      * [Main example](#main-example)
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

This SAW component contains code for interfacing with the Sensable Phantom Omni.  It compiles on Windows and Linux.  It has been tested with:
  * Linux Ubuntu 16.04 and 18.04 and Windows
  * Sensable Phantom Omni 1394 but it might work with other haptic devices from Sensable or 3DS (assuming the C API is the same)

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

## Linux installation

The first thing to do is to install the drivers.  The vendor's version is rather old and incompatible with most recent Linux distributions.  You can find plenty of how-tos online to patch the driver installation but these tend to be incomplete.  So we created a nifty little script to do the work:  https://github.com/jhu-cisst-external/phantom-omni-1394-drivers

The Phantom Omni uses a FireWire port to communicate.  When connecting your tracker to your computer, a pseudo device will be added to the `/dev` directory.   Usually something like `/dev/fw1`, `/dev/fw2` or higher.  Using the command `dmesg -w` can help identify which device is used.  First start `dmesg -w` in a terminal, then plug your device.  You should see something like:
```
[33217.952060] firewire_core 0000:04:00.0: phy config: new root=ffc1, gap_count=5
[33224.507217] firewire_core 0000:04:00.0: phy config: new root=ffc1, gap_count=5
[33224.691773] firewire_core 0000:04:00.0: created device fw1: GUID 000b990082b6980c, S400
```
Check the file permissions on said device, e.g.,
```sh
crw------- 1 root root    243, 0 Mar 22 14:10 /dev/fw0
crw-rw---- 1 root plugdev 243, 1 Mar 23 18:19 /dev/fw1
```
By default the drivers are configured so the OS sets the ownership of `/dev/fw1` to `root` and the group to `plugdev`.   To grant permissions to read and write to the device, use the command `sudo adduser <user_id> plugdev` to add users to the `plugdev` group.   Please note that the user has to logout/login for the new group membership to take effect.

## Configuration

It is important to first run the Sensable provided configuration tool to make sure your device is properly detected and to name it (names are important if you have more than one Omni on the same computer).   To do so, you will have to run the `PHANToMConfiguration` program with admin privileges:
```sh
sudo PHANToMConfiguration
```
In the application, select the *PHANToM Model*.  You should see a serial number appear on the bottom right if everything is working fine.  Click *Add...* and then give your device a name (e.g. "left").  Don't forget to *Apply* before quitting using *Ok*.

In theory, one should be able to have multiple Phantom Omnis on a single computer but so far we were not able to do this on Linux.   If you do figure out a solution, please let us know.   As a stop gap solution, one can use ROS as middleware and 2 computers, one for each Omni.

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

Note: urdf and models provided in this repository are from https://github.com/fsuarez6/phantom_omni

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



## Network based Touch (Geomagic/3Dsystems Touch)

Used as reference for reported joint positions and updated Omni URDF model.

Joints:
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

### Installation

Remove older drivers and OpenHaptics for FireWire based devices (there might be ways to have both installed simultaneously but not sure).

Driver install and configuration:  https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/Installation+Instructions.pdf

Open Haptics v3.4 link: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US

Notes:
* In `~/.bashrc`, add something like `export GTDD_HOME=/usr/share/3DSystems`.  It's strange because installer also creates a rule in `/etc/profile.d/openhaptics.sh` to set  `OH_SDK_BASE`.
* In directory `/usr/share/3DSystems/config`, the program `Touch_Setup` creates files for each arm.