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

It is important to first run the Sensable provided configuration tool to make sure your device is properly detected and to name it.   To do so, you will have to run the `PHANToMConfiguration` program with admin privileges:
```sh
sudo PHANToMConfiguration
```
In the application, select the *PHANToM Model*.  You should see a serial number appear on the bottom right if everything is working fine.  Click *Add...* and then give your device a name (e.g. "left").  Don't forget to *Apply* before quitting using *Ok*.

In theory, one should be able to have multiple Phantom Omnis on a single computer but so far we were not able to do this on Linux.   If you do figure out a solution, please let us know.   As a stop gap solution, one can use ROS as middleware and 2 computers, one for each Omni. 

## Compilation

This code is part of the cisst-SAW libraries and components.  Once the drivers are installed, you can follow the *cisst-SAW* compilation instructions: https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake.

For Linux users, we strongly recommend to compile with ROS and the python catkin build tools (i.e. `catkin build`, NOT `catkin_make`).  Detailled instructions can be found on https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros.

## Main example

The main example provided is `sawSensablePhantomQtExample`.  The command line options are:
```sh
sawSensablePhantomQtExample:
 -j <value>, --json-config <value> : json configuration file (required)
 -m, --component-manager : JSON files to configure component manager (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
```

For example:
```sh
sawSensablePhantomQtExample -D -j sawSensablePhantomLeft.json
```

The JSON configuration file is required since there is no easy way to find out the name you gave to your Phantom Omni.

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

Please read the section above for the configuration file description.  The ROS node is `sensable_phantom` and can be found in the package `sensable_phantom_ros`:
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
/left/state_command
/stats/events/period_statistics
/stats/left/period_statistics
/stats/publishers_left/period_statistics
/stats/subscribers/period_statistics
```
