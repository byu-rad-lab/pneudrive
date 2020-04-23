#ifndef PCONTROL_H_
#define PCONTROL_H_

#include <vector.h>
#include "I2CDevice.h"

/**
 * @class PressureController
 * @brief Designed to create a ROS node and create an interface with multiple lower level pressure control microcontrollers
 */
class PressureController{
private:
  int numNodes;
  int numPressuresPerNode;
  std::vector<I2CDevice> i2cDevices;
  std::vector<std::vector<float>> pressures;
  std::vector<std::vector<float>> pressureCommands;
  std::vector<std::vector<unsigned char>> pressureChars;
  std::vector<std::vectory<unsigned char>> pressureCommandChars;
  
public:
  PressureController(int bus, int firstAddress);
  void do_pressure_control();
  int get_num_devices_on_bus(int bus)
  float two_bytes_to_float(unsigned char * twobytes);
  void float_to_two_bytes(float myfloat, unsigned char * twobytes);
  void pcmd_callback(const std_msgs::Float32MultiArray msg);
};

#endif /* PCONTROL_H_ */
