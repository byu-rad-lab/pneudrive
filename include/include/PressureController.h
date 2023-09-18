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
#define PSI2KPA 6.8947572932
#define P_MAX 100 * PSI2KPA
#define V_SUP 5.0
#define DEBUG_MODE true

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
  std::vector<int> jointMissedCounter;

  unsigned char incomingDataBytes[BYTES_IN_PACKET-2];
  unsigned char outgoingBytes[BYTES_IN_PACKET];
  unsigned short incomingDataShorts[4] = {0,0,0,0};
  unsigned short outgoingShorts[5] = {0,1,2,3,4};
  
  int fd;
  std::map<std::string, int> rs485_addresses;
  ros::AsyncSpinner spinner;
  double analogToKpa(unsigned short analog);
  unsigned short kpaToAnalog(float kPa);
  void initializeSerial();
  void initializeDataVectors();
  void startSubscribers(ros::NodeHandle n);
  void startPublishers(ros::NodeHandle n);

  bool waitForResponse(int timeoutMillieconds);
  bool handleIncomingBytes(unsigned short jointAddress);
  void prepareOutgoingBytes(unsigned short jointAddress);

public:
  PressureController(ros::NodeHandle n, std::map<std::string, int>& rs485_config);
  ~PressureController();
  void do_pressure_control();
  void ping_devices();
  void shortToBytes(unsigned short *short_array, unsigned char* byte_array);
  void byteToShorts(unsigned short *short_array, unsigned char* byte_array);
  void pcmd_callback(const rad_msgs::PressureStamped::ConstPtr& msg, int joint);
  void publishCallback(const ros::TimerEvent& event);
};

#endif /* PCONTROL_H_ */
