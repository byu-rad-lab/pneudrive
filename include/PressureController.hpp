#ifndef PCONTROL_H_
#define PCONTROL_H_

#include <vector>
#include <rad_msgs/msg/pressure_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h> //for delay function
#include <numeric>

#define BYTES_IN_PACKET 10
#define PSI2KPA 6.8947572932
#define P_MAX 100 * PSI2KPA
#define V_SUP 5.0
#define DEBUG_MODE false

/**
 * @class PressureController
 * @brief Designed to create a ROS2 node and create an interface with multiple lower level pressure control microcontrollers
 */
class PressureController : public rclcpp::Node
{
private:
  int numJoints;
  int numPressuresPerJoint = 4;

  std::mutex m;

  std::vector<std::shared_ptr<rclcpp::Publisher<rad_msgs::msg::PressureStamped>>> pressurePublishers;
  std::shared_ptr<rclcpp::TimerBase> publisher_timer;

  std::vector<std::shared_ptr<rclcpp::Subscription<rad_msgs::msg::PressureStamped>>> pressureCommandSubscribers;
  std::vector<std::vector<float>> pressures;
  std::vector<std::vector<float>> pressureCommands;
  std::vector<int> jointMissedCounter;

  unsigned char incomingDataBytes[BYTES_IN_PACKET - 2];
  unsigned char outgoingBytes[BYTES_IN_PACKET];
  unsigned short incomingDataShorts[4] = { 0, 0, 0, 0 };
  unsigned short outgoingShorts[5] = { 0, 0, 0, 0, 0 };

  int fd;
  std::map<std::string, int> rs485_addresses;
  double analogToKpa(unsigned short analog);
  unsigned short kpaToAnalog(float kPa);
  void initializeSerial();
  void initializeDataVectors();
  void startSubscribers();
  void startPublishers();

  bool waitForResponse(int timeoutMillieconds);
  bool handleIncomingBytes(int joint);
  void prepareOutgoingBytes(int joint);
  float filter(float prev, float input);

public:
  PressureController(int num_joints);
  ~PressureController();
  void do_pressure_control();
  void ping_devices();
  void shortToBytes(unsigned short* short_array, unsigned char* byte_array);
  void byteToShorts(unsigned short* short_array, unsigned char* byte_array);
  void pcmd_callback(const rad_msgs::msg::PressureStamped::SharedPtr msg, int joint);
  void publishCallback();
};

#endif /* PCONTROL_H_ */
