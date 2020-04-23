#include "PressureController.h"

int main(int argc, char **argv)
{
  int bus = 1;
  int firstAddress = 10;
  int pressuresPerNode = 4;
  PressureController controller(bus, firstAddress, pressuresPerNode);

  controller.do_pressure_control();

  return 0;  
}
