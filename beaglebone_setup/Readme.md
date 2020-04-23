# How to set up BeagleBone Black
There are two options for configuring a BeagleBone Black (BBB) to run as the ROS master in our pressure control system:

1. The easiest way is that a preconfigured image can be flashed to the BBB. ***NOTE - WE SHOULD HOST THIS IMAGE SOMEWHERE SINCE IT IS LIKELY TOO BIG TO BE ON THIS GITHUB***. Once this image is downloaded, use the instructions [here](https://linuxpropaganda.wordpress.com/2018/06/26/backup-and-restore-your-beaglebone-black/) to flash it onto the BBB.

2. Another way to configure the BBB is to follow the instructions below for installing Ubuntu on the BBB, then to download this repo onto the BBB and execute the script byu_pressure_control/beaglebone_setup/bb_install_script.sh to finish the setup. This process is detailed below.

## Install Ubuntu
The BBB comes with Debian Linux installed, however we need an OS which is supported by ROS. We choose to use Ubuntu 18.04.

1. Download an Ubuntu 18.04.4 flasher [here](https://rcn-ee.com/rootfs/2020-04-09/flasher/bone-eMMC-flasher-ubuntu-18.04.4-console-armhf-2020-04-09-2gb.img.xz) and extract it using ```unxz <name of the downloaded file>```

2. Install startup disk creator using ```sudo apt install usb-creator-gtk```

3. Insert a MicroSD card through a USB adapter into your computer

4. Use startup disk creator to specify that you want the file you just downloaded and extracted on the MicroSD card you inserted

5. Insert the MicroSD card into the BBB, then connect power while holding the S2 button. Keep holding the S2 button until you see the lights on the BBB start to move left and right. This means that the EMMC memory on the BBB is being written to. After it is done it will power off. This usually takes 5-6 minutes.

The BBB now has Ubuntu installed. Remove the SD card.

## Download and execute install script

Now that Ubuntu has been installed you can plug in a keyboard, HDMI, and an ethernet cable connection for internet. The default username and password are "ubuntu" and "temppwd" respectively. Log in and download this repo using

```git clone https://bitbucket.org/byu_rad_lab/byu_pressure_control.git```.

Navigate to beaglebone_setup and execute the install script using

```./bb_install_script.sh```.

This should:

* Install ROS and other needed packages
* Enable I2C using the BBB's GPIO pins
* Set a static IP address on boot (by default this address is 192.168.0.188)
* Start the pressure control node on boot

This should all be done automatically using the install script, but below are more details for the insatiably curious.

## Install ROS and other needed packages
Use the instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS melodic. Do not install ros-melodic-desktop-full because we won't use any ROS GUI tools. Instead install ros-melodic-ros-base.

After finishing the ROS installation, install some more packages upon which we will depend later:

```sudo apt install nano ros-melodic-robot-upstart ros-melodic-tf```

## Enable I2C using GPIO pins

Simply place the lines

```uboot_overlay_addr6=/lib/firmware/BB-I2C1-FAST-00A0.dtbo```
and
```uboot_overlay_addr6=/lib/firmware/BB-I2C1-FAST-00A0.dtbo```
in the file ```/boot/uEnv.txt``` in order to enable fast I2C on the I2C1 and I2C2 busses of the BBB. You must reboot in order for these to take effect.


## Run Pressure Controller on boot
We use the robot_upstart package in order to start our pressure controller on boot. This package installs any ROS launch file as a service which is executed on boot. In order to install our pressure control launch file as a service we execute the following commands (for reference or details fon installing/uninstalling, look [here](http://docs.ros.org/jade/api/robot_upstart/html/)).

```rosrun robot_upstart install byu_pressure_control/launch/i2c_pressure_control.launch --user root```

## Set IP address on boot
Place a file in ```/etc/init.d/``` called ```set_ip_on_startup```. This is a shell script which will be executed on startup, but has to have a specific format like the one in beaglebone_setup. In this file, we choose to execute a python script within the repo. We did this so it is easier to find and change this IP address later on. Also, we found that adding a time delay to the command to change the IP address was important. Without this time delay, the IP address was not consistently changed. After placing ```set_ip_on_startup``` and the python script ```setIPOnStartup.py``` where they belong, execute
```sudo update-rc.d set_ip_on_startup defaults```
```sudo update-rc.d set_ip_on_startup enable```
in order for the ```init.d``` shell script to be executed on boot.

