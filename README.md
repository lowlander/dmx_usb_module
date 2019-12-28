# DMX USB Driver

The DMX USB driver is a Linux 4.X kernel driver for the Enttec
DMX USB dongle ( http://www.enttec.com/dmxusb.php )

## Building the driver

Before the driver can be build, the kernel sourcecode needs to be 
installed. To build the driver just call make, this should build a
`dmx_usb.ko` (the kernel module) and a `dmx_usb_test` (a small test
program).

## Building on a RPi4 running Raspbian

First update your RPi with the following commands;

```
apt-get update -y
apt-get upgrade -y
```

After that you might want to reboot to make sure you are running the
maybe updated kernel.

Than get the kernel source code;

```
# Get rpi-source
sudo wget https://raw.githubusercontent.com/notro/rpi-source/master/rpi-source -O /usr/bin/rpi-source

# Make it executable
sudo chmod +x /usr/bin/rpi-source

# Tell the update mechanism that this is the latest version of the script
/usr/bin/rpi-source -q --tag-update

# Get the kernel files thingies.
rpi-source
```

Now you should be ready to build the DMX USB module.

## Problems loading the right driver.

A "small" problem to solve was to make the kernel clear that it should 
use my driver for the dongle and not the standard usbserial driver. 
Since I don't have any other USBserial devices with the samechip type I 
just removed the `ftdi_sio.ko` module, and changed the `/etc/hotplug/usb.distmap` 
to point to my module. To support both the `ftdi_sio.o` module and the 
`dmx_usb.o` module the `ftdi_sio.o` module should be patched and the Product ID 
for the FT232BM should be removed from it, this will mean all other 
serial-usb devices are still supported just not the one with FT232BM chips, 
since it isn't possible to differentiate DMX-USB and normal-serial versions.

