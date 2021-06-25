#include <sstream>
#include "PressureController.h"

// conversion factor from psi to kpa (ie kpa = psi * psi2kpa)
const double psi2kpa = 6.8947572932;

PressureController::PressureController(ros::NodeHandle n, int bus, std::map<std::string, int> expected_i2c_addresses)
{
  /* 
  * Set up GPIO pins. One digital input is needed for checking if estop has been hit.  
  * One digital output is needed to control power to arduinos via power relay strip.  
  * Power to arduinos is normally off, so need to set output high to enable them. 
  */
  pwrEnable.setDirection(GPIO::OUTPUT);
  pwrEnable.setValue(GPIO::HIGH);
  ROS_INFO_STREAM("ENABLED ARDUINO POWER");
  usleep(1000000); //sleep for 1 second to allow arduinos to turn on fully.

  //check to make sure expected devices are found on i2c bus
  check_devices_on_bus(bus, expected_i2c_addresses);

  // get number of expected devices to make vectors the right size
  numJoints = expected_i2c_addresses.size();

  i2cDevices.resize(numJoints);
  pressures.resize(numJoints);
  pressureCommands.resize(numJoints);

  for (int i = 0; i < numJoints; i++)
  {
    std::string joint_name = "joint_" + std::to_string(i);
    i2cDevices[i].open(bus, expected_i2c_addresses[joint_name]);
    pressures[i].resize(numPressuresPerJoint);
    pressureCommands[i].resize(numPressuresPerJoint);
  }

  for (int i = 0; i < numJoints; i++)
  {
    std::string topicString = "/robo_0/joint_" + std::to_string(i) + "/pressure_command";
    /*
      See https://answers.ros.org/question/63991/how-to-make-callback-function-called-by-several-subscriber/?answer=63998?answer=63998#post-id-63998 for more details on this trickery.
     */
    ros::Subscriber sub = n.subscribe<byu_pressure_control::PressureStamped>(topicString, 1000, boost::bind(&PressureController::pcmd_callback, this, _1, i));
    pressureCommandSubscribers.push_back(sub);
    ROS_INFO("/pressure_command topic started for joint %d", i);
  }

  for (int i = 0; i < numJoints; i++)
  {
    std::string topic_string = "/robo_0/joint_" + std::to_string(i) + "/pressure_state";
    ros::Publisher pub = n.advertise<byu_pressure_control::PressureStamped>(topic_string, 1000);
    pressurePublishers.push_back(pub);
    ROS_INFO("/pressure_state topic started for joint %d", i);
  }
}

void PressureController::check_devices_on_bus(int bus, std::map<std::string, int> expected_i2c_addresses)
{
  ROS_INFO_STREAM("Scanning for i2c devices on bus " << bus << "...");

  std::vector<bool> found_i2c_device;

  for (int i = 0; i < expected_i2c_addresses.size(); i++)
  {
    I2CDevice device;
    std::string joint_name = "joint_" + std::to_string(i);
    int addr = expected_i2c_addresses[joint_name];
    unsigned char testchar;

    device.open(bus, addr);
    bool error = device.readRegisters(0, sizeof(testchar), &testchar);

    if (!error)
    {
      found_i2c_device.push_back(true);
      ROS_INFO_STREAM("Found device 0x" << std::hex << addr << " on bus " << bus);
    }
    else
    {
      found_i2c_device.push_back(false);
      ROS_INFO_STREAM("Could not find device 0x" << std::hex << addr << " on bus " << bus);
    }
  }

  int checksum = std::accumulate(found_i2c_device.begin(), found_i2c_device.end(), 0);

  if (checksum != expected_i2c_addresses.size())
  {
    ROS_ERROR_ONCE("Not all i2c devices found. Shutting down node.");
    ros::shutdown();
  }
}

void PressureController::do_pressure_control()
{
  while (ros::ok())
  {

    ROS_INFO_STREAM_ONCE("PRESSURE CONTROL STARTED");

    // update memory values for  pressure commands and pressures
    for (int joint = 0; joint < numJoints; joint++)
    {
      unsigned char pchar[numPressuresPerJoint * 2];
      for (int p = 0; p < numPressuresPerJoint; p++)
      {
        float_to_two_bytes(pressureCommands[joint][p], &pchar[p * 2]);
      }

      i2cDevices[joint].writeRegisters(0, sizeof(pchar), &pchar[0]);

      bool error = i2cDevices[joint].readRegisters(0, sizeof(pchar), &pchar[0]);

      if (error)
      {
        // send error message over ROS once per second on i2c failure
        ROS_ERROR_STREAM_THROTTLE(1, "I2C Failure on node " << joint);
      }

      for (int p = 0; p < numPressuresPerJoint; p++)
      {
        pressures[joint][p] = two_bytes_to_float(&pchar[p * 2]);
      }

      //This section just prints things out the cout for debugging purposes.
      //	  std::cout<<"\n\nNode "<<node<<" commands:"<<std::endl;
      //	   for(int p=0; p<numPressuresPerNode; p++)
      //	     {
      //	       std::cout<<pressureCommands[node][p]<<"  ";
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
      byu_pressure_control::PressureStamped msg;
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

  //disable power to arduinos
  pwrEnable.setValue(GPIO::LOW);
}

float PressureController::two_bytes_to_float(unsigned char *twobytes)
{
  uint16_t myint;
  memcpy(&myint, twobytes, 2);
  //std::cout <<  "myint: " << myint << std::endl;
  float myfloat = (float)(100.0 * myint * psi2kpa / 65535.0);
  //std::cout << "Pressure: " << myfloat << std::endl;
  return myfloat;
}

void PressureController::float_to_two_bytes(float myfloat, unsigned char *twobytes)
{
  //std::cout << "Pressure command: " << myfloat << std::endl;
  uint16_t myint = (myfloat / (100.0 * psi2kpa)) * 65535.0;
  memcpy(twobytes, &myint, 2);
}

void PressureController::pcmd_callback(const byu_pressure_control::PressureStamped::ConstPtr &msg, int joint)
{
  for (int i = 0; i < msg->pressure.size(); i++)
  {
    float temp = (float)msg->pressure[i]; // cast double/float64 to float/float32 to send over i2c
    pressureCommands[joint][i] = temp;
  }
}
