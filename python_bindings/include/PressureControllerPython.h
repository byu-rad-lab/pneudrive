#ifndef PCONTROLPYTHON_H_
#define PCONTROLPYTHON_H_

#include <vector>
#include <unistd.h> //for delay function
#include <iostream>
#include <numeric>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#define BYTES_IN_PACKET 10
#define PSI2KPA 6.8947572932
#define P_MAX 100 * PSI2KPA
#define V_SUP 5.0
#define DEBUG_MODE false

namespace py = pybind11;

/**
 * @class PressureControllerPython
 * @brief Designed to create a ROS node and create an interface with multiple lower level pressure control microcontrollers
 */
class PressureControllerPython
{
private:
  int numJoints;
  int numPressuresPerJoint = 4;

  std::vector<std::vector<float>> pressures;
  std::vector<std::vector<float>> pressureCommands;
  std::vector<int> jointMissedCounter;

  unsigned char incomingDataBytes[BYTES_IN_PACKET - 2];
  unsigned char outgoingBytes[BYTES_IN_PACKET];
  unsigned short incomingDataShorts[4] = {0, 0, 0, 0};
  unsigned short outgoingShorts[5] = {0, 0, 0, 0, 0};

  int fd;
  std::map<std::string, int> rs485_addresses;
  double analogToKpa(unsigned short analog);
  unsigned short kpaToAnalog(float kPa);
  void initializeSerial(const char *port);
  void initializeDataVectors();

  void update(int joint);
  bool waitForResponse(int timeoutMillieconds);
  bool handleIncomingBytes(int joint);
  void prepareOutgoingBytes(int joint);
  float filter(float prev, float input);
  void shortToBytes(unsigned short *short_array, unsigned char *byte_array);
  void byteToShorts(unsigned short *short_array, unsigned char *byte_array);

public:
  PressureControllerPython(const char *port, int numDevices);
  ~PressureControllerPython();

  void setPressureCommands(int deviceNum, std::vector<float> &pressureCommands);
  std::vector<float> &getPressureData(int deviceNum);
  void ping_devices();
};

#endif /* PCONTROLPYTHON_H_ */
