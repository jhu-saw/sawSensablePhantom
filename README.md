# sawSensablePhantom


# 3DS USB Touch on Linux

Tested on Ubuntu 16.04
 * Ubuntu 16.04 (I also tried a 14.04 VM)
 * geomagic_touch_device_driver_2016.1-1-amd64

## Testing that the device is connected

Kernel logs:
```sh
dmesg
```
returns:
```
[ 2687.594607] usb 2-1.1: new full-speed USB device number 10 using ehci-pci
[ 2687.689233] usb 2-1.1: New USB device found, idVendor=0301, idProduct=2988
[ 2687.689246] usb 2-1.1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 2687.689249] usb 2-1.1: Product: 3D Systems Touch 3D Stylus
[ 2687.689252] usb 2-1.1: Manufacturer: 3D Systems
[ 2687.689254] usb 2-1.1: SerialNumber: 00000000050C
[ 2687.689672] cdc_acm 2-1.1:1.0: ttyACM0: USB ACM device
```

Listing USB devices:
```sh
lsusb
```
shows a device with the proper vendor id and product id:
```
Bus 002 Device 005: ID 413c:1004 Dell Computer Corp. 
Bus 002 Device 012: ID 0301:2988  
Bus 002 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
```

The device handle should properly created as "/dev/ttyACM0".

Permissions by default grant read/write access to user root and group dialout.   DO NOT use the recommended `chmod 777` on the device node as recommended in the install manual, you will have to do this as root/su after each reboot and each time you unplug the device. Instead, add your users to the dialout group:
```
sudo adduser <user-id> dialout
```
The `adduser` command will be effective after next login.

## Installing the drivers

The file to use is geomagic_touch_device_driver_2016.1-1-amd64.tgz. Extract and then run the install.sh as sudo.

After the install, some cleanup is required:
 * edit the file `/etc/profile.d/geomagic.sh`, comment out the last two exports (`LD_LIBRARY_PATH` and `QT_PLUGIN_PATH`).  Qt 5 should be installed using Ubuntu packages.
 * In the directory `/opt/geomagic_touch_device_drivers`
   * `rm -r lib`
   * `chmod 755 .`
   * `chmod 755 *`
   * `chmod 777 config`.  That's not ideal but this will allow users to create new configurations.
   * `chmod 644 */*`

On a 16.04 machine, we had slightly better results by simply plugging the USB cable on the back of our PC. Said PC is a bit old (Dell Optiplex 990) and it seems that not all USB ports are equal. Using a different port we were able to detect the Touch and get a model/serial number using the Setup application. When running the Diagnostic application as well as the deviceQuery example, it seems that the "driver" is sometimes able to read a single sample from the USB but the data doesn't get refreshed afterwards.

On a 14.04 host, we were able to run the Setup and Diagnostic application without any issue.

Please note that the cisst/SAW component doesn't use OpenHaptics.
