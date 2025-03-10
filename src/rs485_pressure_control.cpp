#include <iostream>
#include "PressureController.hpp"

int main(int argc, char** argv)
{
  // Initialize the node 
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("PressureController");

  // Create a parameter for numjoints to handle it as an input via command line
  node->declare_parameter<int>("numjoints");
  int num_joints; 

  // Verify numjoints was inputted and save it to num_joints
  if (node->get_parameter("numjoints", num_joints))
  {
    RCLCPP_INFO(node->get_logger(), "Retrieved parameter numjoints: %d", num_joints);
    // Set node to spin until shutdown
    rclcpp::spin(std::make_shared<PressureController>(num_joints));
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to retrieve rs485 information from parameter server.");
  }
  rclcpp::shutdown();

  return 0;
}
