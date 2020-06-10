#include <iostream>
#include "IMUPublisher.h"

int main(int argc, char **argv)
{
  int bus = 2;
  int firstAddress = 50;
  ros::init(argc, argv, "IMUPublisher");

  IMUPublisher publisher(bus, firstAddress);

  publisher.run();

  return 0;  
}
