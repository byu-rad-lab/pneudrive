#include <iostream>
#include "PressureController.hpp"

int main(int argc, char** argv)
{

  // start up node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("PressureController");

  std::map<std::string, int> expected_rs485_addresses;

  if (node->getParam("rs485_bus_addresses", expected_rs485_addresses))
  {
    PressureController controller(node, expected_rs485_addresses);
    controller.do_pressure_control();
  }
  else
  {
    RCLCPP_ERROR("Failed to retrieve rs485 information from parameter server.");
  }

  return 0;
}
