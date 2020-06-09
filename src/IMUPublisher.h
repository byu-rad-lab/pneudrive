#ifndef IMUPUBLISHER_H_
#define IMUPUBLISHER_H_

#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
#include "I2CDevice.h"

/**
 * @class IMUPublisher
 * @brief Designed to create a ROS node and create an interface with multiple lower level microcontrollers each doing pose estimation based on an IMU
 */
class IMUPublisher{
private:
  int numNodes;
  std::vector<ros::Publisher> imuPublishers;
  std::vector<I2CDevice> i2cDevices;
  std::vector<std::vector<float>> poses;
  
  
public:
  IMUPublisher(int bus, int firstAddress);
  void run();
  int get_num_devices_on_bus(int bus, int firstAddress);
  float two_bytes_to_float(unsigned char * twobytes);
  void float_to_two_bytes(float myfloat, unsigned char * twobytes);
};

#endif /* IMUPUBLISHER_H_ */
