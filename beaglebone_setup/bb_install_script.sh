#!/bin/sh

# INSTALL ROS and needed packages

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

apt-get update

sudo apt install ros-melodic-ros-base -y

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

apt-get install nano ros-melodic-robot-upstart ros-melodic-tf -y

# Create a ROS Workspace and put this package in it
cd
mkdir -p ~/ros_ws/src
cd ros_ws
catkin_make
cd src
git clone https://bitbucket.org/byu_rad_lab/byu_pressure_control.git
cd ..
catkin_make

# Enable I2C on the BeagleBoneBlack

echo "uboot_overlay_addr6=/lib/firmware/BB-I2C1-FAST-00A0.dtbo" >> /boot/uEnv.txt

echo "uboot_overlay_addr7=/lib/firmware/BB-I2C2-FAST-00A0.dtbo" >> /boot/uEnv.txt

