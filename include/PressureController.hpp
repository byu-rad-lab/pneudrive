#ifndef PCONTROL_H_
#define PCONTROL_H_

#include <vector>
#include <rad_msgs/msg/pressure_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
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
  int num_joints;
  int num_pressures_per_joint = 4;

  std::thread control_thread;
  std::mutex mutex_lock;

  std::vector<std::shared_ptr<rclcpp::Publisher<rad_msgs::msg::PressureStamped>>> pressure_publishers;
  std::shared_ptr<rclcpp::TimerBase> publisher_timer;

  std::vector<std::shared_ptr<rclcpp::Subscription<rad_msgs::msg::PressureStamped>>> pressure_command_subscribers;
  std::vector<std::vector<float>> pressures;
  std::vector<std::vector<float>> pressure_commands;
  std::vector<int> joint_missed_counter;

  unsigned char incoming_data_bytes[BYTES_IN_PACKET - 2];
  unsigned char outgoing_bytes[BYTES_IN_PACKET];
  unsigned short incoming_data_shorts[4] = { 0, 0, 0, 0 };
  unsigned short outgoing_shorts[5] = { 0, 0, 0, 0, 0 };

  int fd;
  std::map<std::string, int> rs485_addresses;
  double analog_to_kpa(unsigned short analog);
  unsigned short kpa_to_analog(float kPa);
  void initialize_serial();
  void initialize_data_vectors();
  void start_subscribers();
  void start_publishers();

  bool wait_for_response(int timeout_millieconds);
  bool handle_incoming_bytes(int joint);
  void prepare_outgoing_bytes(int joint);
  float filter(float prev, float input);

public:
  PressureController(int num_joints);
  ~PressureController();
  void serial_loop();
  void ping_devices();
  void short_to_bytes(unsigned short* short_array, unsigned char* byte_array);
  void byte_to_shorts(unsigned short* short_array, unsigned char* byte_array);
  void pcmd_callback(const rad_msgs::msg::PressureStamped::SharedPtr msg, int joint);
  void publish_callback();
};

#endif /* PCONTROL_H_ */
