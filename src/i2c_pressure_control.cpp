#include "PressureController.h"

int main(int argc, char **argv)
{
  int bus = 1;
  int firstAddress = 10;
  PressureController::PressureController controller(bus, firstAddress);

  controller.do_pressure_control();

  return 0;  
}
