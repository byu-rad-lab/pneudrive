# How to set up BeagleBone Black
Below are the instructions for configuring a BeagleBone Black (BBB) to run as the ROS master in our pressure control system. Alternatively, a preconfigured image can be flashed to the BBB which obviates the need for the rest of these instructions. ***NOTE - WE SHOULD HOST THIS IMAGE SOMEWHERE SINCE IT IS LIKELY TOO BIG TO BE ON THIS GITHUB***. Once this image is downloaded, use the instructions [here](https://linuxpropaganda.wordpress.com/2018/06/26/backup-and-restore-your-beaglebone-black/) to flash it onto the BBB.

## Install Ubuntu
The BBB comes with Debian Linux installed, however we need an OS which is supported by ROS. We choose to use Ubuntu 18.04.

Follow the instructions [here](https://linuxpropaganda.wordpress.com/2018/06/26/install-ubuntu-server-on-beaglebone-black/) for installing Ubuntu 18.04 on the BeagleBone Black.

## Enable I2C using GPIO pins

## Install ROS and needed ROS packages
Use the instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS melodic. Do not install ros-melodic-desktop-full because we won't use any ROS GUI tools. Instead install ros-melodic-ros-base.

After finishing the ROS installation, install some more packages upon which we will depend later:

```sudo apt install ros-melodic-robot-upstart```

## Run Pressure Controller on boot
We use the robot_upstart package in order to start our pressure controller on boot. This package installs any ROS launch file as a service which is executed on boot. In order to install our pressure control launch file as a service we execute the following commands (for reference or details fon installing/uninstalling, look [here](http://docs.ros.org/jade/api/robot_upstart/html/)).

```rosrun robot_upstart install byu_pressure_control/launch/i2c_pressure_control.launch --user root```

## Set IP address on boot
