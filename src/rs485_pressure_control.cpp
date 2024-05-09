#include <iostream>
#include "PressureController.hpp"

int main(int argc, char **argv)
{

  // start up node
  ros::init(argc, argv, "PressureController");
  ros::NodeHandle n;

  std::map<std::string, int> expected_rs485_addresses;


  if (n.getParam("/rs485_bus_addresses", expected_rs485_addresses))
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
