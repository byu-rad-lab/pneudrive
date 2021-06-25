#ifndef PCONTROL_H_
#define PCONTROL_H_

#include <vector>
#include <rad_msgs/PressureStamped.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
#include "I2CDevice.h"
#include "GPIO.h"
#include <ros/console.h>
#include <unistd.h> //for delay function
#include <numeric>

/**
 * @class PressureController
 * @brief Designed to create a ROS node and create an interface with multiple lower level pressure control microcontrollers
 */
class PressureController
{
private:
  int numJoints;
  int numPressuresPerJoint = 4;
  GPIO::GPIO pwrEnable{49}; //P9_23, output to enable power to arduinos and valves
  std::vector<ros::Publisher> pressurePublishers;
  std::vector<ros::Subscriber> pressureCommandSubscribers;
  std::vector<I2CDevice> i2cDevices;
  std::vector<std::vector<float>> pressures;
  std::vector<std::vector<float>> pressureCommands;

public:
  PressureController(ros::NodeHandle n, int bus, std::map<std::string, int> expected_i2c_addresses);
  void do_pressure_control();
  void check_devices_on_bus(int bus, std::map<std::string, int> expected_i2c_addresses);
  float two_bytes_to_float(unsigned char *twobytes);
  void float_to_two_bytes(float myfloat, unsigned char *twobytes);
  void pcmd_callback(const rad_msgs::PressureStamped::ConstPtr &msg, int joint);
};

#endif /* PCONTROL_H_ */
