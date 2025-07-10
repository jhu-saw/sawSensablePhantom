# sawSensablePhantom

This SAW component contains code for interfacing with the SensAble *PHANTOM Omni* and the Geomagic/3DS *Touch*.  It compiles on Windows and Linux.  It has been tested with:
  * Linux Ubuntu 18.04 to 24.04 and Windows
  * SensAble *Phantom Omni* (FireWire/1394) and 3DS *Touch* (Ethernet) but it might work with other haptic devices from SensAble or 3DS (assuming the C API is the same)

The `ros` folder contains code for a ROS node that interfaces with the
sawSensablePhantom component and publishes the 3D transformations,
joint positions and efforts, buttons...  It also has subscribers to
control the Omni (i.e. set cartesian wrench).  To build the ROS node,
make sure you use `catkin build`.

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/main/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Sensable SDK and drivers, see below
 * Qt for user interface
 * ROS (optional, ROS 1 or ROS 2)

# Running the examples

## Drivers

On windows, follow the instructions from the vendor.  On Linux, please read [drivers.md](drivers.md)

## Compilation

This code is part of the cisst-SAW libraries and components.  Once the drivers are installed, you can follow the *cisst-SAW* compilation instructions: https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake.

For Linux users, we strongly recommend to compile with ROS (1 or 2).  See https://github.com/jhu-saw/vcs for download and build instructions.  Use the VCS files for `sensable`.

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

The ROS node is `sensable_phantom` and can be found in the package `sensable_phantom`.  If you only have one Omni and don't need a configuration file:
```
rosrun sensable_phantom sensable_phantom
```

If you have more than one Omni, please read the section above for the configuration file description.
```sh
roscd sensable_phantom_config
rosrun sensable_phantom sensable_phantom -D 
```

The ROS node has one more command line option:
```sh
 -j <value>, --json-config <value> : json configuration file (optional)
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
roslaunch sensable_phantom rviz.launch
```

If the arm has a different name (e.g. `right`), use:
```sh
roslaunch sensable_phantom rviz.launch name:=right
```

You can also start both the sensable node and RViz using:
```sh
roslaunch sensable_phantom omni_rviz.launch
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
