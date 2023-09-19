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

PressureController::~PressureController()
{
  serialFlush(this->fd);
  serialClose(this->fd);
}

void PressureController::ping_devices()
{
  serialFlush(this->fd);
  ROS_INFO("Checking communication with serial devices...");

  for (int joint = 0; joint < numJoints; joint++)
  {
    unsigned short jointAddress = this->rs485_addresses["joint_" + std::to_string(joint)];
    ROS_INFO("Pinging joint %d", jointAddress);

    prepareOutgoingBytes(joint);

    ssize_t numBytesWritten = write(this->fd, this->outgoingBytes, BYTES_IN_PACKET);

    bool timeout = waitForResponse(5000);
    
    if(!timeout)
    {
      if (handleIncomingBytes(joint))
      {
        ROS_INFO_STREAM("Joint " << jointAddress << " ping successful.");
      }
      else
      {
        ROS_WARN("Unsucessful read");
        ROS_ERROR_ONCE("Not all devices found. Killing node.");
        ros::shutdown();
      }
    }
    else
    {
      ROS_ERROR_ONCE("Not all devices found. Killing node.");
      ros::shutdown();
    }
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
    printf("Loop num: %d\n", numLoops);


    for (int joint = 0; joint < numJoints; joint++)
    {
      if(DEBUG_MODE)
      {
        // ros::Duration(.03).sleep();
      }
      
      serialFlush(fd); // clear the current serial buffer before doing anything with it, RX AND TX



      prepareOutgoingBytes(joint);

      if(DEBUG_MODE)
      {
        printf("Pressure commands: ");
        for (int i=0;i<4;i++)
        {
          printf("%f ", this->pressureCommands[joint][i]);
        }
        printf("\n");

        printf("Address + outgoing pressure command shorts: ");
        for (int i=0;i<5;i++)
        {
          printf("%d ", this->outgoingShorts[i]);
        }
        printf("\n");

        printf("Address + outgoing pressure command bytes: ");
        for (int i=0;i<10;i++)
        {
          printf("%02x ", this->outgoingBytes[i]);
        }
        printf("\n");
      }

      if (write(this->fd, this->outgoingBytes, BYTES_IN_PACKET) != 10)
      {
        ROS_WARN("Incorrect amount of bytes sent.");
      }

      bool timeout = waitForResponse(2);

      if(!timeout)
      {
        if(handleIncomingBytes(joint))
        {
          //convert analog shorts to kpa and load for sending over ROS
          for (size_t i=0;i<numPressuresPerJoint;i++)
          {
            float tmp = analogToKpa(this->incomingDataShorts[i]);
            this->pressures[joint][i] = filter(this->pressures[joint][i], tmp);
          }

          if(DEBUG_MODE)
          {
            printf("Converted to pressures: ");
            for (int i=0;i<4;i++)
            {
              printf("%f ", this->pressures[joint][i]);
            }
            printf("\n");
          }

          //reset contact counter for this joint since communication was succesful
          jointMissedCounter[joint] = 0;
        }
        else
        {
          printf("Unsucessful read. Data not saved.\n");
        }
      }
      else
      {
        numMissed++;
        jointMissedCounter[joint]++;

        if (jointMissedCounter[joint] > 10)
        {
          ROS_ERROR_STREAM("Lost connection with joint " << joint);
        }
        else
        {
          ROS_WARN("Joint %d: Communication response timeout", joint);
        }

        //empty both RX and TX buffers
        serialFlush(fd);
      }
    }

    ros::Duration loop_time = ros::Time::now() - loop_start;
    if (loop_time > max_loop_time)
    {
      max_loop_time = loop_time;
    }

    numLoops++;

    if(DEBUG_MODE)
    {
      ROS_INFO_STREAM("Loop Time: " << ros::Time::now() - loop_start << " s");
      std::cout << std::endl;
    }
  }

  std::cout << "Max loop time was " << max_loop_time << std::endl;
  std::cout << "Dropped " << numMissed << " of " << numLoops << std::endl;
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
  // convert bin number to a voltage
  double v_out = analog * 5.0 / 1024.0;

  // return applied pressure in kPa
  return ((v_out - 0.1 * V_SUP) / (0.8 * V_SUP)) * P_MAX;
}

unsigned short PressureController::kpaToAnalog(float kPa)
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
  jointMissedCounter.resize(numJoints);



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

bool PressureController::waitForResponse(int timeoutMilliseconds)
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

bool PressureController::handleIncomingBytes(int joint)
{
  unsigned short jointAddress = this->rs485_addresses["joint_" + std::to_string(joint)];
  unsigned char firstByte = 0;
  bool readSuccessful = false;


  while (serialDataAvail(fd) > 0)
  {
    if(DEBUG_MODE)
    {
      std::cout << "Bytes available on handle: " << std::dec << serialDataAvail(fd) << std::endl;
    }

    unsigned char secondByte = serialGetchar(fd);

    if(DEBUG_MODE)
    {
      printf("First Byte: %02x", firstByte);
      printf(" Second Byte: %02x\n", secondByte);
    }

    unsigned short short1 = (firstByte << 8) | secondByte;

    if(DEBUG_MODE)
    {
      printf("short to check: %d\n", short1);
    }

    if(short1 == jointAddress)
    {
      if(read(this->fd, this->incomingDataBytes, BYTES_IN_PACKET-2) == BYTES_IN_PACKET-2)
      {
        byteToShorts(this->incomingDataShorts, this->incomingDataBytes);

        if(DEBUG_MODE)
        {
          std::cout << "Address match!" <<std::endl;
          std::cout << "Incoming bytes: ";
          for (int i = 0; i < 8; i++)
          {
            printf("%02x ", incomingDataBytes[i]);
          }
          std::cout << std::endl;

          std::cout << "Converted to shorts: ";
          for (int i = 0; i < 4; i++)
          {
            printf("%d ", incomingDataShorts[i]);
          }
          std::cout << std::endl;

          printf("Bytes available after read: %d\n", serialDataAvail(fd));
        }
        
        // check that valid data was received
        for (int i=0; i<4; i++)
        {
          //must be limited by 10 bit resolution of ADC, otherwise this is incorrect
          if(this->incomingDataShorts[i] > 1023)
          {
            readSuccessful=false;
            printf("Invalid shorts recieved. Should be 0-1023. Throwing away data.\n");
            break;
          }
          else
          {
            readSuccessful=true;
          }
        }
        firstByte = 0;
      }
    }
    else
    {
      if(DEBUG_MODE)
      {
        std::cout << "No address match" << std::endl;
      }
      firstByte = secondByte;
    }
  }

  return readSuccessful;
}

void PressureController::prepareOutgoingBytes(int joint)
{
  // Construct a byte message to send to the arduino
  // assign both bytes of address to first two bytes of outgoing packet
  unsigned short jointAddress = this->rs485_addresses["joint_" + std::to_string(joint)];
  this->outgoingShorts[0] = jointAddress;

  //fill appropriate commands from this->pressure_commands into outgoing Shorts
  for (int i=0;i<4;i++)
  {
    this->outgoingShorts[i+1] = kpaToAnalog(this->pressureCommands[joint][i]);
  }

  shortToBytes(this->outgoingShorts, this->outgoingBytes); // converts test_short_write (shorts) to check_msg_write (bytes)
}

float PressureController::filter(float prev, float input)
{  /*
      This function implements a first order low pass filter
      with a cutoff frequency of 50 Hz. First order hold discrete implementation with dt=.001.
      First order filter is of form:
      a / (z - b)
  */
  float a = 1.0;
  float b = 1.0 - a;

  return b * prev + a * input;
}
