#include <iostream>
#include "PressureController.hpp"

int main(int argc, char** argv)
{

  // start up node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("PressureController");
  node->declare_parameter("numjoints", 4);
  //rclcpp::spin(node);

  int num_joints; 

  if (node->get_parameter("numjoints", num_joints))
  {
    RCLCPP_INFO(node->get_logger(), "YAY! Retrieved parameter numjoints: %d", num_joints);
    PressureController controller(node, num_joints);
    controller.do_pressure_control();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to retrieve rs485 information from parameter server.");
  }

  return 0;
}
