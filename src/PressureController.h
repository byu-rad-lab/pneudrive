#ifndef PCONTROL_H_
#define PCONTROL_H_

#include <vector>
#include <rad_msgs/PressureStamped.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
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
  std::vector<ros::Publisher> pressurePublishers;
  std::vector<ros::Subscriber> pressureCommandSubscribers;
  std::vector<std::vector<float>> pressures;
  std::vector<std::vector<float>> pressureCommands;

public:
  PressureController(ros::NodeHandle n, std::map<std::string, int> expected_rs485_addresses);
  void do_pressure_control();
  void check_devices_on_bus(int bus, std::map<std::string, int> expected_rs485_addresses);
  void shortToBytes(unsigned short *short_array, unsigned char *byte_array);
  void byteToShorts(unsigned short *short_array, unsigned char *byte_array);
  void pcmd_callback(const rad_msgs::PressureStamped::ConstPtr &msg, int joint);
};

#endif /* PCONTROL_H_ */
