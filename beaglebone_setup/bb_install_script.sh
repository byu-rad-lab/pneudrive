#!/bin/sh

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

apt-get update

sudo apt install ros-melodic-ros-base -y

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

apt-get install nano ros-melodic-robot-upstart ros-melodic-tf -y

echo "uboot_overlay_addr6=/lib/firmware/BB-I2C1-FAST-00A0.dtbo" >> /boot/uEnv.txt

echo "uboot_overlay_addr7=/lib/firmware/BB-I2C2-FAST-00A0.dtbo" >> /boot/uEnv.txt
