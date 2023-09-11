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


PressureController::PressureController(ros::NodeHandle n, std::map<std::string, int>& rs485_config)
  : rs485_addresses(rs485_config), spinner(3)
{

  initializeSerial();
  initializeDataVectors();
  this->ping_devices();

  spinner.start();

  startSubscribers(n);
  startPublishers(n);
}

void PressureController::ping_devices()
{
  serialFlush(this->fd);
  ROS_INFO("Checking communication with serial devices...");
  bool error_flag = false;

  for (int joint = 0; joint < numJoints; joint++)
  {
    // specify target arduino
    //todo: check how this int gets cast into unsigned short
    unsigned short jointAddress = this->rs485_addresses["joint_" + std::to_string(joint)];

    // Construct a byte message to send to the arduino
    // assign both bytes of address to first two bytes of outgoing packet
    this->outgoingShorts[0] = jointAddress;

    ROS_INFO("Pinging joint %d", jointAddress);
    shortToBytes(this->outgoingShorts, this->outgoingBytes); // converts test_short_write (shorts) to check_msg_write (bytes)

    // Write byte message to the arduino
    ssize_t numBytesWritten = write(this->fd, this->outgoingBytes, BYTES_IN_PACKET);

    ros::Time start = ros::Time::now();

    // Wait for arduino to respond with 10 bytes
    while (serialDataAvail(fd) != BYTES_IN_PACKET)
    {
      // Add time check error here
      if (ros::Time::now() - start > ros::Duration(1))
      {
        ROS_ERROR("Joint %d: Communication response timeout", joint);
        error_flag = true;
        break;
      }
    }
    

    // READ the response from the arduino
    if(!error_flag)
    {
      ssize_t numBytesRead = read(this->fd, incomingBytes, BYTES_IN_PACKET);
      ROS_INFO("Joint %d: Communication successful", joint);
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
  ros::Duration max_loop_time(0);
  int numLoops = 0;
  int numMissed = 0;
  int lost_contact_counter = 0;
  while (ros::ok())
  {
    ros::Time loop_start = ros::Time::now();
    ROS_INFO_STREAM_ONCE("PRESSURE CONTROL STARTED");

    serialFlush(fd); // clear the current serial buffer before doing anything with it

    for (int joint = 0; joint < numJoints; joint++)
    {
      // std::this_thread::sleep_for(std::chrono::microseconds(1)); //this seems to be necessary for some reason, devices hit time out without it. 

      bool error_flag = false;
      // specify target arduino
      //todo: check int cast to ushort
      unsigned short jointAddress = this->rs485_addresses["joint_" + std::to_string(joint)];

      // assign both bytes of address to first two bytes of outgoing packet
      this->outgoingShorts[0] = jointAddress;
      // this->outgoingShorts[1] = 2;

      //prepare commands received over ROS for sending to arduinos
      for (int i=0;i<4;i++)
      {
        // this->outgoingShorts[i+1] = kpaToAnalog(this->pressureCommands[joint][i]);
        this->outgoingShorts[i+1] = i;
      }

      shortToBytes(this->outgoingShorts, this->outgoingBytes); // converts test_short_write (shorts) to check_msg_write (bytes)

      // Write byte message to the arduino
      if (write(this->fd, this->outgoingBytes, BYTES_IN_PACKET) != 10)
      {
        ROS_WARN("Incorrect amount of bytes sent.");
      }

      // Capture the start time
      // auto start = std::chrono::high_resolution_clock::now();
      ros::Time start = ros::Time::now();

      // Wait for arduino to respond with 10 bytes
      while (serialDataAvail(fd) != BYTES_IN_PACKET)
      {
        // std::cout << "waiting for data" << std::endl;
        // Add time check error here

        //there's a fair bit of nondeterminism that happens when bits get into TX buffer. If it happens that this
        // takes too long, move on to other joints to not delay everything. 
        if (ros::Time::now() - start > ros::Duration(.002))
        {
          error_flag = true;
          break;
        }
      }

      // READ the response from the arduino
      if(!error_flag)
      {
        ssize_t numBytesRead = read(this->fd, incomingBytes, BYTES_IN_PACKET);

        byteToShorts(this->incomingShorts, this->incomingBytes);

        //convert analog shorts to kpa and load for sending over ROS
        for (size_t i=0;i<numPressuresPerJoint;i++)
        {
          float tmp = analogToKpa(this->incomingShorts[i+1]);
          this->pressures[joint][i] = tmp;
        }

        //reset contact counter for this joint since communication was succesful
        jointContactCounter[joint] = 0;
      }
      else
      {
        //its possible there are some bytes in the RX buffer... so empty it out before reading next one
        numMissed++;
        jointContactCounter[joint]++;

        if (jointContactCounter[joint] > 10)
        {
          ROS_WARN_STREAM("Lost connection with joint " << joint);
        }

        while(serialDataAvail(fd)>0)
        {
          int trash = serialGetchar(fd);
        }
      }
    }

    ros::Duration loop_time = ros::Time::now() - loop_start;
    if (loop_time > max_loop_time)
    {
      max_loop_time = loop_time;
    }

    // ROS_INFO_STREAM("Loop Time: " << ros::Time::now() - loop_start << " s");
    numLoops++;
  }

  std::cout << "Max loop time was " << max_loop_time << std::endl;
  std::cout << "Threw away " << numMissed << " of " << numLoops;
}

void PressureController::publishCallback(const ros::TimerEvent& event)
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

//todo: move to utils header
void PressureController::shortToBytes(unsigned short* short_array, unsigned char* byte_array)
{
  // Function to convert array of 5 shorts to array of 10 bytes
  int shortLength = 5;
  for (int i = 0; i < shortLength; i++)
  {
    int byteIndex = i * 2;
    unsigned char* bytePtr = (unsigned char *)& short_array[i];

    // convert from big endian to little endian by switching bytes around
    byte_array[byteIndex] = bytePtr[1];     // LSB
    byte_array[byteIndex + 1] = bytePtr[0]; // MSB
  }
}
 
//todo: move to utils header
void PressureController::byteToShorts(unsigned short* short_array, unsigned char* byte_array)
{
  // Function to convert array of 10 bytes to array of 5 shorts
  unsigned int byteLength = 10;
  for (size_t i = 0; i < byteLength; i += 2)
  {
    short_array[i / 2] = ((short)byte_array[i] << 8) | byte_array[i+1];
  }
}

double PressureController::analogToKpa(unsigned short analog)
{
  /*
     Function to convert an analog pressure reading into kPa
     ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
     Analog reference is 5V, so 0=0v and 1023=5v
     Conversion is taken from Fig. 3 (transfer function A) in pressure sensor datasheet
     https://www.mouser.com/datasheet/2/187/honeywell-sensing-basic-board-mount-pressure-abp-s-1224358.pdf
  */
  // conversion factor from psi to kpa (ie kpa = psi * PSI2KPA)
  double const PSI2KPA = 6.8947572932;
  double const P_MAX = 100 * PSI2KPA;

  double v_sup = 5;

  // convert bin number to a voltage
  double v_out = analog * 5.0 / 1024.0;

  // return applied pressure in kPa
  return ((v_out - 0.1 * v_sup) / (0.8 * v_sup)) * P_MAX;
}

unsigned short PressureController::kpaToAnalog(float kPa)
{
  /*
     Function to convert a kPa pressure reading into an analog pressure
     ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
     Analog reference is 5V, so 0=0v and 1023=5v
     This is the inverse of the function "analogToKpa" directly above.
  */
  // conversion factor from psi to kpa (ie kpa = psi * PSI2KPA)
  double const PSI2KPA = 6.8947572932;
  double const P_MAX = 100 * PSI2KPA;

  double v_sup = 5;

  // convert kPa to a voltage
  double v_out = v_sup * ((0.8 * kPa / P_MAX) + 0.1);
  
  // convert voltage to a bin number
  return v_out * 1024.0 / 5.0;
}

void PressureController::initializeSerial()
{
  if ((this->fd = serialOpen("/dev/ttyS1", 1000000)) < 0)
  {
    fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
  }

  if (wiringPiSetup() == -1)
  {
    fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
  }
}

void PressureController::initializeDataVectors()
{
  // get number of expected devices to make vectors the right size
  numJoints = this->rs485_addresses.size();
  pressures.resize(numJoints);
  pressureCommands.resize(numJoints);
  jointContactCounter.resize(numJoints);



  for (int i = 0; i < numJoints; i++)
  {
    // std::string joint_name = "joint_" + std::to_string(i);
    pressures[i].resize(numPressuresPerJoint);
    pressureCommands[i].resize(numPressuresPerJoint);
  }
}

void PressureController::startSubscribers(ros::NodeHandle n)
{
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
}

void PressureController::startPublishers(ros::NodeHandle n)
{
  // Create pressure data publisheers
  for (int i = 0; i < numJoints; i++)
  {
    std::string topic_string = "/robo_0/joint_" + std::to_string(i) + "/pressure_state";
    ros::Publisher pub = n.advertise<rad_msgs::PressureStamped>(topic_string, 1);
    pressurePublishers.push_back(pub);
    ROS_INFO("/pressure_state topic started for joint %d", i);
  }
  
  this->publisher_timer = n.createTimer(ros::Duration(0.002), &PressureController::publishCallback, this);
}

void PressureController::pcmd_callback(const rad_msgs::PressureStamped::ConstPtr &msg, int joint)
{
  for (int i = 0; i < msg->pressure.size(); i++)
  {
    float temp = (float)msg->pressure[i]; // cast double/float64 to float/float to send over i2c
    pressureCommands[joint][i] = temp;
  }
}
