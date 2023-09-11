#ifndef PCONTROL_H_
#define PCONTROL_H_

#include <vector>
#include <rad_msgs/PressureStamped.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <unistd.h> //for delay function
#include <numeric>

#define BYTES_IN_PACKET 10

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
  ros::Timer publisher_timer;
  
  std::vector<ros::Subscriber> pressureCommandSubscribers;
  std::vector<std::vector<float>> pressures;
  std::vector<std::vector<float>> pressureCommands;
  std::vector<int> jointContactCounter;

  unsigned char incomingBytes[BYTES_IN_PACKET];
  unsigned char outgoingBytes[BYTES_IN_PACKET];
  unsigned short incomingShorts[5] = {0,0,0,0,0};
  unsigned short outgoingShorts[5] = {0,0,0,0,0};
  
  int fd;
  std::map<std::string, int> rs485_addresses;
  ros::AsyncSpinner spinner;
  double analogToKpa(unsigned short analog);
  unsigned short kpaToAnalog(float kPa);
  void initializeSerial();
  void initializeDataVectors();
  void startSubscribers(ros::NodeHandle n);
  void startPublishers(ros::NodeHandle n);

public:
  PressureController(ros::NodeHandle n, std::map<std::string, int>& rs485_config);
  void do_pressure_control();
  void ping_devices();
  void shortToBytes(unsigned short *short_array, unsigned char* byte_array);
  void byteToShorts(unsigned short *short_array, unsigned char* byte_array);
  void pcmd_callback(const rad_msgs::PressureStamped::ConstPtr& msg, int joint);
  void publishCallback(const ros::TimerEvent& event);
};

#endif /* PCONTROL_H_ */
