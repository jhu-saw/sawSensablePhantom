# sawSensablePhantom


# 3DS USB Touch on Linux

Tested on Ubuntu 16.04
- Ubuntu 16.04 (I also tried a 14.04 VM)
- geomagic_touch_device_driver_2016.1-1-amd64
- openhaptics_3.4-0-developer-edition-amd64

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

Permissions by default grant read/write access to user root and group dialout.  To add a user to the dialout group:
```
sudo adduser <user-id> dialout
```
The adduser command will be effective after next login.


```
TO BE CLEANED UP
(which I am a member of). I nevertheless chmod the ttyACM0 to 777. So far so good but when I start " /opt/geomagic_touch_device_driver/Geomagic_Touch_Setup" (without sudo), I get "Device Serial Num: No USB device available". Device Model is "Touch", Interface is "USB Port" and Port Num is "/dev/ttyACM0".
I poked around a bit and everything seems fine, i.e. my environment variables have:

OH_SDK_BASE=/opt/OpenHaptics/Developer/3.4-0
GTDD_HOME=/opt/geomagic_touch_device_driver
OH_SDK_BASE=/opt/OpenHaptics/Developer/3.4-0

On a 16.04 machine, we had slightly better results by simply plugging the USB cable on the back of our PC. Said PC is a bit old (Dell Optiplex 990) and it seems that not all USB ports are equal. Using a different port we were able to detect the Touch and get a model/serial number using the Setup application. When running the Diagnostic application as well as the deviceQuery example, it seems that the "driver" is able to read a single sample from the USB and the data doesn't get refreshed afterwards. We'll investigate on a faster machine, maybe just a CPU/bandwidth issue.

On a 14.04 host, we were able to run the Setup and Diagnostic application without any issue. A couple of comments though:

The libraries copied in /opt/geomagic_touch_device_drivers might conflict with updated libraries. In our case, we have a fair amount of Qt5 code that uses an updated version of Qt5. Since the installer modifies the LD_LIBRARY_PATH for the whole system, that lead to unresolved symbols. To solve this, you can either remove/comment the LD_LIBRARY_PATH in /etc/profile.d/geomagic.sh or remove all the libraries in /opt/geomagic_touch_device_drivers/lib.
```
