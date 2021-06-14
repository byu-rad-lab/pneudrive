#include <iostream>
#include "PressureController.h"

int main(int argc, char **argv)
{

  // start up node
  ros::init(argc, argv, "PressureController");
  ros::NodeHandle n;

  int bus;
  std::map<std::string, int> expected_i2c_addresses;

  if (n.getParam("/hardware/pressure_sensors/i2c_info/addresses", expected_i2c_addresses) &&
      n.getParam("/hardware/pressure_sensors/i2c_info/i2c_bus", bus))
  {
    PressureController controller(n, bus, expected_i2c_addresses);
    controller.do_pressure_control();
  }
  else
  {
    ROS_ERROR("Failed to retrieve i2c information from parameter server.");
  }

  return 0;
}
