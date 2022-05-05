# Introduction

In theory OpenHaptics should support many devices across different OSs but in practice it's not always easy to find and install the drivers for your haptic devices.  We focus primarily on Linux but the code should work on Windows as well.  On Linux, we tested a limited number of devices:
* SensAble *PHANTOM Omni*, i.e. the original system with a FireWire interface.  Works on Ubuntu 16.04, 18.04 and 20.04.
* Ethernet based *Touch*.  These systems have been sold under the brands *Geomagic* and *3DS*.  Works on Ubuntu 16.04, 18.04 and 20.04.
* USB based *Touch*.  This are even more recent and we believe were all from *3DS*.  Works on Ubuntu 16.04 and 18.04.  We never got these to work on Ubuntu 20.04.  If these worked for you, let us know.
* USB based Geomagic *Touch Stylus*

# SensAble *PHANTOM Omni*

This is the original Omni and many labs have some of these collecting dust on shelves.  These systems use a FireWire interface.  FireWire is a bit dated but it's still possible to find cheap adapters for modern desktop computers (~$50).  It's a bit more challenging to find support for laptops.

## Drivers

If you have used the same computer with the 3DS drivers for the Ethernet based *Touch*, **remove these drivers**.  There is a couple of scripts provided in this repository to remove the drivers and OpenHaptics (in directory 'utils').

The main issue with the SensAble *Omni* is that the drivers for a modern Linux are hard to find.   The vendor's version is rather old and incompatible with most recent Linux distributions.  You can find plenty of how-tos online to patch the driver installation but these tend to be incomplete.  So we created a nifty little script to do the work:  https://github.com/jhu-cisst-external/phantom-omni-1394-drivers

The Phantom Omni uses a FireWire port to communicate.  When connecting your device to your computer, a pseudo device will be added to the `/dev` directory.   Usually something like `/dev/fw1`, `/dev/fw2` or higher.  Using the command `dmesg -w` can help identify which device is used.  First start `dmesg -w` in a terminal, then plug your device.  You should see something like:
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
By default the drivers are configured so the OS sets the ownership of `/dev/fw1` to `root` and the group to `plugdev`.  To grant permissions to read and write to the device, use the command `sudo adduser <user_id> plugdev` to add users to the `plugdev` group.   Please note that the user has to logout/login for the new group membership to take effect.

## Configuration

It is important to first run the SensAble provided configuration tool to make sure your device is properly detected and to name it (names are important if you have more than one Omni on the same computer).   To do so, you will have to run the `PHANToMConfiguration` program with admin privileges:
```sh
sudo PHANToMConfiguration
```

:warning: If you get an error related to `libglw`, you should install the package `libglw1-mesa` (for Ubuntu 18, name might be different on other distributions).

In the application, select the *PHANToM Model*.  You should see a serial number appear on the bottom right if everything is working fine.  Click *Add...* and then give your device a name (e.g. "left").  Don't forget to *Apply* before quitting using *Ok*.

In theory, one should be able to have multiple Phantom Omnis on a single computer but so far we were not able to do this on Linux.   If you do figure out a solution, please let us know.   As a stop gap solution, one can use ROS as middleware and 2 computers, one for each Omni.

# Geomagic/3DS Ethernet *Touch*

The Geomagic/3DS looks like the original PHANTOM Omni except that the base is a couple of inches taller and has Ethernet ports instead of FireWire.  The drivers from the vendor (3DS) should be used along their version of OpenHaptics (3.4 on Linux).  Some of these devices might come with a USB based Ethernet adapter so they can be used with a USB port.  At the end, the actual communication is performed over an Ethernet port configured with *Link-Local*.

## Drivers

If you have used the same computer for a FireWire based Omni, **remove the older drivers and OpenHaptics for FireWire based devices**.  There is an `uninstall` script that comes with the FireWire drivers from https://github.com/jhu-cisst-external/phantom-omni-1394-drivers.

### Automated installation

We created a nifty little script to do the work: https://github.com/jhu-cisst-external/3ds-touch-openhaptics

### Manual installation

* Linux page with links to Open Haptics v3.4 and Touch Device Driver v2019.2.15: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US
* Linux instructions: https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/Installation+Instructions.pdf

### Removing the drivers

Removing files:
* To remove the 3DS provided SDK, use script `utils/uninstall-3ds-openhaptics-3.4.sh` from this repository
* To remove the 3DS provided drivers (`2019_2_15`) use the script `utils/uninstall-3ds-touch-2019.sh` from this repository

## Configuration

Use the provided `Touch_Setup` and `Touch_Diagnostic` to configure and test your device.

# Geomagic *Touch Stylus*

We got no luck getting this running, including with the 2019 3DS drivers.  See older notes in [3DS_USB_Touch.md](3DS_USB_Touch.md) if you want to give it a try.
