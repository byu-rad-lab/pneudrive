#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include "PressureController.h"

#include <wiringPi.h>
#include <wiringSerial.h>

// conversion factor from psi to kpa (ie kpa = psi * psi2kpa)
const double psi2kpa = 6.8947572932;

const int BYTES_PER_PRESSURE = 2;

PressureController::PressureController(ros::NodeHandle n, std::map<std::string, int> expected_rs485_addresses)
{
  
  int fd; // file descriptor for the serial port

  if ((fd = serialOpen("/dev/ttyS1", 1000000)) < 0)
  {
    fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    return 1;
  }

  if (wiringPiSetup() == -1)
  {
    fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
    return 1;
  }
  

  // get number of expected devices to make vectors the right size
  numJoints = expected_rs485_addresses.size();

  pressures.resize(numJoints);
  pressureCommands.resize(numJoints);

  for (int i = 0; i < numJoints; i++)
  {
    // std::string joint_name = "joint_" + std::to_string(i);
    pressures[i].resize(numPressuresPerJoint);
    pressureCommands[i].resize(numPressuresPerJoint);
  }

  // Create pressure command subscribers
  for (int i = 0; i < numJoints; i++)
  {
    std::string topicString = "/robo_0/joint_" + std::to_string(i) + "/pressure_command";
    /*
      See https://answers.ros.org/question/63991/how-to-make-callback-function-called-by-several-subscriber/?answer=63998?answer=63998#post-id-63998 for more details on this trickery.
     */
    ros::Subscriber sub = n.subscribe<rad_msgs::PressureStamped>(topicString, 1, boost::bind(&PressureController::pcmd_callback, this, _1, i),ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    pressureCommandSubscribers.push_back(sub);
    ROS_INFO("/pressure_command topic started for joint %d", i);
  }

  // Create pressure data publisheers
  for (int i = 0; i < numJoints; i++)
  {
    std::string topic_string = "/robo_0/joint_" + std::to_string(i) + "/pressure_state";
    ros::Publisher pub = n.advertise<rad_msgs::PressureStamped>(topic_string, 1000);
    pressurePublishers.push_back(pub);
    ROS_INFO("/pressure_state topic started for joint %d", i);
  }
}

void PressureController::check_devices_on_bus(int bus, std::map<std::string, int> expected_rs485_addresses)
{
  // ===*Some sort of code here to verify everything is connected properly and ready to start communication*===


    // ROS_INFO_STREAM("Scanning for rs485 devices on bus " << bus << "...");

    // std::vector<bool> found_rs485_device;

    // for (int i = 0; i < expected_rs485_addresses.size(); i++)
    // {
    //   rs485Device device;
    //   std::string joint_name = "joint_" + std::to_string(i);
    //   int addr = expected_rs485_addresses[joint_name];
    //   unsigned char testchar;

    //   device.open(bus, addr);
    //   bool error = device.readRegisters(0, sizeof(testchar), &testchar);

    //   if (!error)
    //   {
    //     found_rs485_device.push_back(true);
    //     ROS_INFO_STREAM("Found device 0x" << std::hex << addr << " on bus " << bus);
    //   }
    //   else
    //   {
    //     found_rs485_device.push_back(false);
    //     ROS_INFO_STREAM("Could not find device 0x" << std::hex << addr << " on bus " << bus);
    //   }
    // }

    // int checksum = std::accumulate(found_rs485_device.begin(), found_rs485_device.end(), 0);

    // if (checksum != expected_rs485_addresses.size())
    // {
    //   ROS_ERROR_ONCE("Not all rs485 devices found. Shutting down node.");
    //   ros::shutdown();
    // }
}

void PressureController::do_pressure_control()
{
  while (ros::ok())
  {

    ROS_INFO_STREAM_ONCE("PRESSURE CONTROL STARTED");

    serialFlush(fd); // clear the current serial buffer before doing anything with it

    for (int joint = 0; joint < numJoints; joint++)
    {
      // Not entirely sure if these are in the right spot.
      unsigned char pressure_msg_write[9];
      unsigned char pressure_msg_read[9];
      unsigned short analog_short_write;
      unsigned short analog_short_read;
      char arduino_address_write;
      char arduino_address_read;
      
      // Convert pressureCommands to analog bin values
      analog_short_write = kpaToAnalog(pressureCommands)
      
      // Construct a byte message to send to the arduino
      pressure_msg_write[0] = arduino_address_write;  // first byte dictates which joint
      shortToBytes(analog_short_write, pressure_msg_write); // converts pressureCommands (shorts) to pressure_msg_write (bytes)
      
      // Write byte message to the arduino
      for (int i = 0; i < 9; i++)
      {
        serialPutchar(fd, pressure_msg_write[i]); // write one byte at a time
        //delayMicroseconds(20);
      }

      // Wait for arduino to respond with 9 bytes
      while (serialDataAvail(fd) != 9);

      // READ the response from the arduino
      // printf("\nReceived Arduino Response:");
      int pos = 0;
      while (serialDataAvail(fd))
      {
        pressure_msg_read[pos] = serialGetchar(fd);
        pos++;
        fflush(stdout);
      }
      
      // Error if didn't read correct arduino response
      if (pos != 9)
      {
        printf("\nGot %i bytes. Expected 9.", pos);
        // send error message over ROS once per second on rs485 failure
        ROS_ERROR_STREAM_THROTTLE(1, "rs485 Failure on node " << joint);
      }

      // Print the arduino response in bytes
      printf("\n");
      for (int i = 0; i < pos; i++) 
      {
        printf("%02x ", pressure_msg_read[i]);
      }

      // Convert the response from the arduino (bytes) into short array
      byteToShorts(analog_short_read, pressure_msg_read);
      arduino_address_read = pressure_msg_read[0];

      // Convert response data (in analog voltage readings) to kPa
      pressures[joint] = analogToKPA(analog_short_read)

      // Print the arduino address and corresponding pressure commands as unsigned short integers
      printf("\n");
      printf("%c ", arduino_address_read);
      for (int i = 0; i < 4; i++)
      {
        printf("%hu ", pressures[joint][i]);
      }

      // What remains of this for-loop is whats left after the necessary changes.
      // Probably should confirm if what follows is useful.

      if (error)
      {
        // send error message over ROS once per second on rs485 failure
        ROS_ERROR_STREAM_THROTTLE(1, "rs485 Failure on node " << joint);
      }

      //This section just prints things out the cout for debugging purposes.
      //	  std::cout<<"\n\nNode "<<joint<<" commands:"<<std::endl;
      //	   for(int p=0; p<numPressuresPerJoint; p++)
      //	     {
      //	       std::cout<<pressureCommands[joint][p]<<"  ";
      //	     }
      //	   std::cout<<"\nNode "<<node<<" pressures:"<<std::endl;
      //	   for(int p=0; p<numPressuresPerNode; p++)
      //	     {
      //	       std::cout<<pressures[node][p]<<"  ";
      //	     }
    }

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

    ros::spinOnce();
  }

}

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

void PressureController::byteToShorts(unsigned short *short_array, unsigned char *byte_array)
{
  // Function to convert array of 8 bytes to array of 4 shorts (states start at i=1)
  unsigned int byteLength = 8;
  for (unsigned int i = 0; i < byteLength; i += 2)
  {
    short_array[i / 2] = ((short)byte_array[i + 1] << 8) | byte_array[i + 2];
  }
}

void PressureController::analogToKpa(unsigned short *analog)
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

void PressureController::kpaToAnalog(unsigned short *kPa)
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
  double v_out = v_sup * ((0.8 * kPa / P_MAX) + 0.1)

  // convert voltage to a bin number
  return v_out * 1024.0 / 5.0
}

void PressureController::pcmd_callback(const rad_msgs::PressureStamped::ConstPtr &msg, int joint)
{ 
  for (int i = 0; i < msg->pressure.size(); i++)
  {
    float temp = (float)msg->pressure[i]; // cast double/float64 to float/float32 to send over i2c
    pressureCommands[joint][i] = temp;
  }
}
