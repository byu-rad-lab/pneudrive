#!/bin/bash

# INSTALL ROS and needed packages

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update

sudo apt install ros-melodic-ros-base -y

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source /opt/ros/melodic/setup.bash

sudo apt-get install nano ros-melodic-robot-upstart ros-melodic-tf -y

# Set IP Address on boot
sudo cp set_ip_on_startup /etc/init.d/
sudo update-rc.d set_ip_on_startup defaults
sudo update-rc.d set_ip_on_startup enable

# Create a ROS Workspace and put this package in it
#cd ..
#rm -rf ./byu_pressure_control/
cd ~/
mkdir -p ~/ros_ws/src
cd ros_ws/src
git clone https://bitbucket.org/byu_rad_lab/byu_pressure_control.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc


# Enable I2C on the BeagleBoneBlack

echo "uboot_overlay_addr6=/lib/firmware/BB-I2C1-FAST-00A0.dtbo" >> /boot/uEnv.txt
echo "uboot_overlay_addr7=/lib/firmware/BB-I2C2-FAST-00A0.dtbo" >> /boot/uEnv.txt

# Launch pressure control and/or IMU on boot
#rosrun robot_upstart install byu_pressure_control/launch/i2c_imu.launch --user root --job imu_publisher
#sudo systemctl daemon-reload 
#sudo systemctl start imu_publisher


rosrun robot_upstart install byu_pressure_control/launch/i2c_pressure.launch --user root --job pressure_control
sudo systemctl daemon-reload 
sudo systemctl start pressure_control
