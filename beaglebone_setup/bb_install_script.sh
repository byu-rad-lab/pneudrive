#!/bin/bash

# INSTALL ROS and needed packages

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl -y

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt-get update

sudo apt install ros-noetic-ros-base -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

sudo apt-get install nano ros-noetic-tf -y

# Set ethernet port to be static IP address
#  TODO

# Create a ROS Workspace and put this package in it
cd ~/
mkdir -p ~/ros_ws/src
cd ros_ws/src
git clone https://bitbucket.org/byu_rad_lab/byu_pressure_control.git
git clone https://bitbucket.org/byu_rad_lab/rad_msgs.git
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


#rosrun robot_upstart install byu_pressure_control/launch/i2c_pressure.launch --user root --job pressurecontrol
#sudo systemctl daemon-reload 
#sudo systemctl start pressurecontrol
