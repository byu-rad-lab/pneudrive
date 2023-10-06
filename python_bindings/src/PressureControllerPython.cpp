#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include <chrono>
#include "PressureControllerPython.h"
#include <unistd.h>
#include <thread>

#include <wiringPi.h>
#include <wiringSerial.h>

PressureControllerPython::PressureControllerPython(const char *port, int numDevices) : numJoints(numDevices)
{
  initializeSerial(port);
  initializeDataVectors();
}

PressureControllerPython::~PressureControllerPython()
{
  serialFlush(this->fd);
  serialClose(this->fd);
}

void PressureControllerPython::ping_devices()
{
  serialFlush(this->fd);
  py::print("Checking communication with serial devices...");

  for (int joint = 0; joint < numJoints; joint++)
  {
    unsigned short jointAddress = 65535 - joint;
    py::print("Pinging joint", jointAddress);

    prepareOutgoingBytes(joint);

    ssize_t numBytesWritten = write(this->fd, this->outgoingBytes, BYTES_IN_PACKET);

    bool timeout = waitForResponse(5000);

    if (!timeout)
    {
      if (handleIncomingBytes(joint))
      {
        py::print("Joint", jointAddress, "ping successful.");
      }
      else
      {
        std::string errorMessage = "Attempted read was unsuccesful. Not all devices found.";
        throw std::runtime_error(errorMessage);
      }
    }
    else
    {
      std::string errorMessage = "Device timeout. Not all devices found.";
      throw std::runtime_error(errorMessage);
    }
  }
}

void PressureControllerPython::update(int joint)
{
  serialFlush(fd); // clear the current serial buffer before doing anything with it, RX AND TX

  prepareOutgoingBytes(joint);

  if (DEBUG_MODE)
  {
    py::print("Pressure commands:");
    for (int i = 0; i < 4; i++)
    {
      py::print(this->pressureCommands[joint][i], " ");
    }
    py::print("\n");

    py::print("Address + outgoing pressure command shorts: ");
    for (int i = 0; i < 5; i++)
    {
      py::print(this->outgoingShorts[i]);
    }
    py::print("\n");

    py::print("Address + outgoing pressure command bytes: ");
    for (int i = 0; i < 10; i++)
    {
      py::print(this->outgoingBytes[i]);
    }
    py::print("\n");
  }

  if (write(this->fd, this->outgoingBytes, BYTES_IN_PACKET) != 10)
  {
    py::print("Incorrect amount of bytes sent.");
  }

  bool timeout = waitForResponse(2);

  if (!timeout)
  {
    if (handleIncomingBytes(joint))
    {
      // convert analog shorts to kpa and load for sending over ROS
      for (size_t i = 0; i < numPressuresPerJoint; i++)
      {
        float tmp = analogToKpa(this->incomingDataShorts[i]);
        this->pressures[joint][i] = filter(this->pressures[joint][i], tmp);
      }

      if (DEBUG_MODE)
      {
        py::print("Converted to pressures: ");
        for (int i = 0; i < 4; i++)
        {
          py::print("%f ", this->pressures[joint][i]);
        }
        py::print("\n");
      }

      // reset contact counter for this joint since communication was succesful
      jointMissedCounter[joint] = 0;
    }
    else
    {
      py::print("Unsucessful read. Data not saved.");
    }
  }
  else
  {
    jointMissedCounter[joint]++;

    if (jointMissedCounter[joint] > 10)
    {
      py::print("Lost connection with joint", joint);
    }
    else
    {
      py::print("Joint", joint, ": Communication response timeout");
    }

    // empty both RX and TX buffers
    serialFlush(fd);
  }
}

// todo: move to utils header
void PressureControllerPython::shortToBytes(unsigned short *short_array, unsigned char *byte_array)
{
  // Function to convert array of 5 shorts to array of 10 bytes
  int shortLength = 5;
  for (int i = 0; i < shortLength; i++)
  {
    int byteIndex = i * 2;
    unsigned char *bytePtr = (unsigned char *)&short_array[i];

    // convert from big endian to little endian by switching bytes around
    byte_array[byteIndex] = bytePtr[1];     // LSB
    byte_array[byteIndex + 1] = bytePtr[0]; // MSB
  }
}

// todo: move to utils header
void PressureControllerPython::byteToShorts(unsigned short *short_array, unsigned char *byte_array)
{
  // Function to convert array of 10 bytes to array of 5 shorts
  unsigned int byteLength = 10;
  for (size_t i = 0; i < byteLength; i += 2)
  {
    short_array[i / 2] = ((short)byte_array[i] << 8) | byte_array[i + 1];
  }
}

double PressureControllerPython::analogToKpa(unsigned short analog)
{
  /*
     Function to convert an analog pressure reading into kPa
     ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
     Analog reference is 5V, so 0=0v and 1023=5v
     Conversion is taken from Fig. 3 (transfer function A) in pressure sensor datasheet
     https://www.mouser.com/datasheet/2/187/honeywell-sensing-basic-board-mount-pressure-abp-s-1224358.pdf
  */
  // convert bin number to a voltage
  double v_out = analog * 5.0 / 1024.0;

  // return applied pressure in kPa
  return ((v_out - 0.1 * V_SUP) / (0.8 * V_SUP)) * P_MAX;
}

unsigned short PressureControllerPython::kpaToAnalog(float kPa)
{
  /*
     Function to convert a kPa pressure reading into an analog pressure
     ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
     Analog reference is 5V, so 0=0v and 1023=5v
     This is the inverse of the function "analogToKpa" directly above.
  */
  // convert kPa to a voltage
  double v_out = V_SUP * ((0.8 * kPa / (P_MAX)) + 0.1);

  // convert voltage to a bin number
  return v_out * 1024.0 / 5.0;
}

void PressureControllerPython::initializeSerial(const char *port)
{
  if ((this->fd = serialOpen(port, 1000000)) < 0)
  {
    std::string errorMessage = "Unable to open serial device: " + std::string(strerror(errno));
    throw std::runtime_error(errorMessage);
  }

  if (wiringPiSetup() == -1)
  {
    std::string errorMessage = "Unable to start WiringPi: " + std::string(strerror(errno));
    throw std::runtime_error(errorMessage);
  }
}

void PressureControllerPython::initializeDataVectors()
{
  // get number of expected devices to make vectors the right size
  pressures.resize(this->numJoints);
  pressureCommands.resize(this->numJoints);
  jointMissedCounter.resize(this->numJoints);

  for (int i = 0; i < this->numJoints; i++)
  {
    // std::string joint_name = "joint_" + std::to_string(i);
    pressures[i].resize(numPressuresPerJoint);
    pressureCommands[i].resize(numPressuresPerJoint);
  }
}

bool PressureControllerPython::waitForResponse(int timeoutMilliseconds)
{
  bool timeout = false;

  std::chrono::milliseconds waitTime(timeoutMilliseconds);
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

  while (serialDataAvail(fd) < BYTES_IN_PACKET)
  {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
    if (elapsed > waitTime)
    {
      timeout = true;
      break;
    }
  }

  return timeout;
}

bool PressureControllerPython::handleIncomingBytes(int joint)
{
  unsigned short jointAddress = 65535 - joint;
  unsigned char firstByte = 0;
  bool readSuccessful = false;

  while (serialDataAvail(fd) > 0)
  {
    if (DEBUG_MODE)
    {
      py::print("Bytes available on handle: ", serialDataAvail(fd));
    }

    unsigned char secondByte = serialGetchar(fd);

    if (DEBUG_MODE)
    {
      py::print("First Byte:", firstByte);
      py::print("Second Byte:", secondByte);
    }

    unsigned short short1 = (firstByte << 8) | secondByte;

    if (DEBUG_MODE)
    {
      py::print("short to check:", short1);
    }

    if (short1 == jointAddress)
    {
      if (read(this->fd, this->incomingDataBytes, BYTES_IN_PACKET - 2) == BYTES_IN_PACKET - 2)
      {
        byteToShorts(this->incomingDataShorts, this->incomingDataBytes);

        if (DEBUG_MODE)
        {
          py::print("Address match!");
          py::print("Incoming bytes:");
          for (int i = 0; i < 8; i++)
          {
            py::print(incomingDataBytes[i]);
          }
          py::print("\n");

          py::print("Converted to shorts: ");
          for (int i = 0; i < 4; i++)
          {
            py::print(incomingDataShorts[i]);
          }
          py::print("\n");

          py::print("Bytes available after read:", serialDataAvail(fd));
        }

        // check that valid data was received
        for (int i = 0; i < 4; i++)
        {
          // must be limited by 10 bit resolution of ADC, otherwise this is incorrect
          if (this->incomingDataShorts[i] > 1023)
          {
            readSuccessful = false;
            py::print("Invalid shorts recieved. Should be 0-1023. Throwing away data.");
            break;
          }
          else
          {
            readSuccessful = true;
          }
        }
        firstByte = 0;
      }
    }
    else
    {
      if (DEBUG_MODE)
      {
        py::print("No address match");
      }
      firstByte = secondByte;
    }
  }

  return readSuccessful;
}

void PressureControllerPython::prepareOutgoingBytes(int joint)
{
  // Construct a byte message to send to the arduino
  // assign both bytes of address to first two bytes of outgoing packet
  unsigned short jointAddress = 65535 - joint;
  this->outgoingShorts[0] = jointAddress;

  // fill appropriate commands from this->pressure_commands into outgoing Shorts
  for (int i = 0; i < 4; i++)
  {
    this->outgoingShorts[i + 1] = kpaToAnalog(this->pressureCommands[joint][i]);
  }

  shortToBytes(this->outgoingShorts, this->outgoingBytes); // converts test_short_write (shorts) to check_msg_write (bytes)
}

float PressureControllerPython::filter(float prev, float input)
{ /*
     This function implements a first order low pass filter
     with a cutoff frequency of 50 Hz. First order hold discrete implementation with dt=.001.
     First order filter is of form:
     a / (z - b)
 */
  float a = 0.6;
  float b = 1.0 - a;

  return b * prev + a * input;
}

void PressureControllerPython::setPressureCommands(int deviceNum, std::vector<float> &pressureCommands)
{
  this->pressureCommands[deviceNum] = pressureCommands;
  this->update(deviceNum);
}

std::vector<float> &PressureControllerPython::getPressureData(int deviceNum)
{
  return this->pressures[deviceNum];
}

// pybind stuff
PYBIND11_MODULE(pneudrive_py, m)
{
  m.doc() = "Pneudrive pressure controller module";
  py::class_<PressureControllerPython>(m, "PressureController")
      .def(py::init<const char *, int>())
      .def("set_pressure_commands", &PressureControllerPython::setPressureCommands, "Send pressure commands to a device")
      .def("get_pressure_data", &PressureControllerPython::getPressureData, "Get pressure data from a device")
      .def("ping_devices", &PressureControllerPython::ping_devices, "Ping all devices to check communication");
}
