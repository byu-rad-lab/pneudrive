#ifndef PCONTROL_H_
#define PCONTROL_H_

#include <vector>
#include <byu_pressure_control/PressureStamped.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
#include "I2CDevice.h"
#include "GPIO.h"
#include <ros/console.h>
#include <unistd.h> //for delay function

/**
 * @class PressureController
 * @brief Designed to create a ROS node and create an interface with multiple lower level pressure control microcontrollers
 */
class PressureController
{
private:
  int numNodes;
  int numPressuresPerNode;
  GPIO::GPIO estopOut{49}; //P9_23
  std::vector<ros::Publisher> pressurePublishers;
  std::vector<ros::Subscriber> pressureCommandSubscribers;
  std::vector<I2CDevice> i2cDevices;
  std::vector<std::vector<float>> pressures;
  std::vector<std::vector<float>> pressureCommands;

public:
  PressureController(ros::NodeHandle n, int bus, int firstAddress, int pressuresPerNode);
  void do_pressure_control();
  int get_num_devices_on_bus(int bus, int firstAddress);
  float two_bytes_to_float(unsigned char *twobytes);
  void float_to_two_bytes(float myfloat, unsigned char *twobytes);
  void pcmd_callback(const byu_pressure_control::PressureStamped::ConstPtr &msg, int node);
};

#endif /* PCONTROL_H_ */
