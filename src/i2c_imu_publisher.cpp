#include <iostream>
#include "IMUPublisher.h"

int main(int argc, char **argv)
{
  int bus = 1;
  int firstAddress = 100;
  IMUPublisher publisher(bus, firstAddress);

  publisher.run();

  return 0;  
}
