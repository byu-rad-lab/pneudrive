#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include <chrono>
#include "PressureController.h"
#include <unistd.h>
#include <thread>

#include <wiringPi.h>
#include <wiringSerial.h>

// conversion factor from psi to kpa (ie kpa = psi * psi2kpa)
const double psi2kpa = 6.8947572932;

const int BYTES_PER_PRESSURE = 2;

PressureController::PressureController(ros::NodeHandle n, std::map<std::string, int> &rs485_config) : rs485_addresses(rs485_config), spinner(3)
{

  if ((this->fd = serialOpen("/dev/ttyS1", 1000000)) < 0)
  {
    fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
  }

  if (wiringPiSetup() == -1)
  {
    fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
  }

  // get number of expected devices to make vectors the right size
  numJoints = this->rs485_addresses.size();
  pressures.resize(numJoints);
  pressureCommands.resize(numJoints);

  this->ping_devices();

  for (int i = 0; i < numJoints; i++)
  {
    // std::string joint_name = "joint_" + std::to_string(i);
    pressures[i].resize(numPressuresPerJoint);
    pressureCommands[i].resize(numPressuresPerJoint);
  }

  spinner.start();

  // Create pressure command subscribers
  for (int i = 0; i < numJoints; i++)
  {
    std::string topicString = "/robo_0/joint_" + std::to_string(i) + "/pressure_command";
    /*
      See https://answers.ros.org/question/63991/how-to-make-callback-function-called-by-several-subscriber/?answer=63998?answer=63998#post-id-63998 for more details on this trickery.
     */
    ros::Subscriber sub = n.subscribe<rad_msgs::PressureStamped>(topicString, 1, boost::bind(&PressureController::pcmd_callback, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    pressureCommandSubscribers.push_back(sub);
    ROS_INFO("/pressure_command topic started for joint %d", i);
  }

  // Create pressure data publisheers
  for (int i = 0; i < numJoints; i++)
  {
    std::string topic_string = "/robo_0/joint_" + std::to_string(i) + "/pressure_state";
    ros::Publisher pub = n.advertise<rad_msgs::PressureStamped>(topic_string, 1);
    pressurePublishers.push_back(pub);
    ROS_INFO("/pressure_state topic started for joint %d", i);
  }

  publisher_timer = n.createTimer(ros::Duration(0.002), &PressureController::publishCallback, this);
}

void PressureController::ping_devices()
{
  serialFlush(this->fd);
  ROS_INFO("Checking communication with serial devices...");
  bool error_flag = false;

  for (int joint = 0; joint < numJoints; joint++)
  {
    // specify target arduino
    unsigned char jointAddress = this->rs485_addresses["joint_" + std::to_string(joint)];

    // specify target message (gives each joint a unique msg if we want to check if arduino can mirror it)
    // todo: check how this int gets cast into unsigned char
    unsigned short test_short_write[4] = {static_cast<unsigned short>(joint * 1000)};

    // Construct a byte message to send to the arduino
    this->outgoingBytes[0] = jointAddress;               // first byte dictates which joint
    shortToBytes(test_short_write, this->outgoingBytes); // converts test_short_write (shorts) to check_msg_write (bytes)

    // Write byte message to the arduino
    ssize_t numBytesWritten = write(this->fd, this->outgoingBytes, BYTES_IN_PACKET);

    // Capture the start time
    static std::chrono::_V2::steady_clock::time_point start = std::chrono::steady_clock::now();

    // Wait for arduino to respond with 9 bytes
    while (serialDataAvail(fd) != 9)
    {
      // Add time check error here
      if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(1000))
      {
        ROS_ERROR("Joint %d: Communication response timeout", joint);
        error_flag = true;
        break;
      }
    }

    // READ the response from the arduino
    if (!error_flag)
    {
      ssize_t numBytesRead = read(this->fd, incomingBytes, BYTES_IN_PACKET);
      ROS_INFO("Joint %d: Communication successful", joint);
    }

    // Check if the read address matches the write address
    if (static_cast<unsigned char>(incomingBytes[0]) == jointAddress)
    {
    }
    else
    {
      ROS_ERROR_STREAM("Joint " << joint << ": Received communication from incorrect device "
                                            "(expected: "
                                << joint << ", received: " << static_cast<unsigned char>(incomingBytes[0]) << ")");
      error_flag = true;
    }
  }

  if (error_flag)
  {
    ROS_ERROR_ONCE("Not all devices found. Killing node.");
    ros::shutdown();
  }
}

void PressureController::do_pressure_control()
{
  while (ros::ok())
  {
    auto arm_time_start = std::chrono::high_resolution_clock::now();
    ROS_INFO_STREAM_ONCE("PRESSURE CONTROL STARTED");

    serialFlush(fd); // clear the current serial buffer before doing anything with it

    for (int joint = 0; joint < numJoints; joint++)
    {

      std::this_thread::sleep_for(std::chrono::microseconds(10)); // this seems to be necessary for some reason, devices hit time out without it.

      bool error_flag = false;
      // specify target arduino
      // todo: check how int gets cast into unsigned char
      unsigned char jointAddress = this->rs485_addresses["joint_" + std::to_string(joint)];

      // specify target message (gives each joint a unique msg if we want to check if arduino can mirror it)
      unsigned short test_short_write[4] = {static_cast<unsigned short>(joint * 1000)};

      // Construct a byte message to send to the arduino
      this->outgoingBytes[0] = jointAddress;               // first byte dictates which joint
      shortToBytes(test_short_write, this->outgoingBytes); // converts test_short_write (shorts) to check_msg_write (bytes)

      // Write byte message to the arduino
      ssize_t numBytesWritten = write(this->fd, this->outgoingBytes, BYTES_IN_PACKET);

      // Capture the start time
      auto start = std::chrono::high_resolution_clock::now();

      // Wait for arduino to respond with 9 bytes
      while (serialDataAvail(fd) != 9)
      {
        // std::cout << "waiting for data" << std::endl;
        // Add time check error here
        if (std::chrono::high_resolution_clock::now() - start > std::chrono::milliseconds(50))
        {
          ROS_WARN_THROTTLE(1, "Joint %d: No response received in 50 ms.", joint);
          error_flag = true;
          break;
        }
      }

      // READ the response from the arduino
      if (!error_flag)
      {
        ssize_t numBytesRead = read(this->fd, incomingBytes, BYTES_IN_PACKET);
      }
    }

    ros::spinOnce();
    // std::cout << "Time Point in Microseconds: " << std::chrono::duration_cast<std::chrono::microseconds>((std::chrono::high_resolution_clock::now() - arm_time_start)).count() << " microseconds" << std::endl;
  }
}

void PressureController::publishCallback(const ros::TimerEvent &event)
{
  // publish pressures
  for (int joint = 0; joint < numJoints; joint++)
  {
    rad_msgs::PressureStamped msg;
    msg.header = std_msgs::Header();
    msg.header.stamp = ros::Time::now();

    msg.pressure.resize(numPressuresPerJoint);

    for (int p = 0; p < numPressuresPerJoint; p++)
    {
      msg.pressure[p] = pressures[joint][p];
    }
    pressurePublishers[joint].publish(msg);
  }
}

// todo: move to utils header
void PressureController::shortToBytes(unsigned short *short_array, unsigned char *byte_array)
{
  // Function to convert array of 4 shorts to array of 8 bytes (doesn't change first byte because of address byte)
  int shortLength = 4;
  for (int i = 0; i < shortLength; i++)
  {
    int byteIndex = i * 2 + 1;
    unsigned char *bytePtr = (unsigned char *)&short_array[i];
    byte_array[byteIndex] = bytePtr[1];     // Most significant byte
    byte_array[byteIndex + 1] = bytePtr[0]; // Least significant byte
  }
}

// todo: move to utils header
void PressureController::byteToShorts(unsigned short *short_array, unsigned char *byte_array)
{
  // Function to convert array of 8 bytes to array of 4 shorts, ignoring first byte
  unsigned int byteLength = 8;
  for (size_t i = 0; i < byteLength; i += 2)
  {
    int byteIndex = i + 1;
    short_array[i / 2] = ((short)byte_array[byteIndex] << 8) | byte_array[byteIndex + 1];
  }
}

// void PressureController::analogToKpa(unsigned short *analog)
// {
//   /*
//      Function to convert an analog pressure reading into kPa
//      ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
//      Analog reference is 5V, so 0=0v and 1023=5v
//      Conversion is taken from Fig. 3 (transfer function A) in pressure sensor datasheet
//      https://www.mouser.com/datasheet/2/187/honeywell-sensing-basic-board-mount-pressure-abp-s-1224358.pdf
//   */
//   // conversion factor from psi to kpa (ie kpa = psi * PSI2KPA)
//   double const PSI2KPA = 6.8947572932;
//   double const P_MAX = 100 * PSI2KPA;

//   double v_sup = 5;

//   // convert bin number to a voltage
//   double v_out = analog * 5.0 / 1024.0;

//   // return applied pressure in kPa
//   return ((v_out - 0.1 * v_sup) / (0.8 * v_sup)) * P_MAX;
// }

// void PressureController::kpaToAnalog(unsigned short *kPa)
// {
//   /*
//      Function to convert a kPa pressure reading into an analog pressure
//      ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
//      Analog reference is 5V, so 0=0v and 1023=5v
//      This is the inverse of the function "analogToKpa" directly above.
//   */
//   // conversion factor from psi to kpa (ie kpa = psi * PSI2KPA)
//   double const PSI2KPA = 6.8947572932;
//   double const P_MAX = 100 * PSI2KPA;

//   double v_sup = 5;

//   // convert kPa to a voltage
//   double v_out = v_sup * ((0.8 * kPa / P_MAX) + 0.1)

//                  // convert voltage to a bin number
//                  return v_out *
//                  1024.0 / 5.0
// }

void PressureController::pcmd_callback(const rad_msgs::PressureStamped::ConstPtr &msg, int joint)
{
  for (int i = 0; i < msg->pressure.size(); i++)
  {
    float temp = (float)msg->pressure[i]; // cast double/float64 to float/float32 to send over i2c
    pressureCommands[joint][i] = temp;
  }
}
