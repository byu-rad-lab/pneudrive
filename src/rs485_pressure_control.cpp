#include <iostream>
#include "PressureController.h"

int main(int argc, char **argv)
{

  // start up node
  ros::init(argc, argv, "PressureController");
  ros::NodeHandle n;

  std::map<std::string, int> expected_rs485_addresses;

  if (n.getParam("/hardware/pressure_sensors/rs485_info/addresses", expected_rs485_addresses))
  {
    PressureController controller(n, expected_rs485_addresses);
    controller.do_pressure_control();
  }
  else
  {
    ROS_ERROR("Failed to retrieve rs485 information from parameter server.");
  }

  return 0;
}
