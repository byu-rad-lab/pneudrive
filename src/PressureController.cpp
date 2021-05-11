#include<sstream>
#include "PressureController.h"

// conversion factor from psi to kpa (ie kpa = psi * psi2kpa)
const double psi2kpa = 6.8947572932;

PressureController::PressureController(ros::NodeHandle n, int bus=1, int firstAddress=10, int pressuresPerNode=4)
{
  /* 
  * Set up GPIO pins. One digital input is needed for checking if estop has been hit.  
  * One digital output is needed to control power to arduinos via power relay strip.  
  * Power to arduinos is normally off, so need to set output high to enable them. 
  */

  estopOut.setDirection(GPIO::OUTPUT);
  estopOut.setValue(GPIO::HIGH);
  ROS_INFO_STREAM("ENABLED ARDUINO POWER");
  usleep(1000000); //sleep for 1 second to allow arduinos to turn on fully.

  numNodes = get_num_devices_on_bus(bus,firstAddress);

  // check to make sure there is at least one node to start, otherwise shut down
  if(numNodes == 0){
    ros::shutdown();
  }
  numPressuresPerNode = pressuresPerNode;
  
  i2cDevices.resize(numNodes);
  pressures.resize(numNodes);
  pressureCommands.resize(numNodes);
  
  for(int i=0; i<numNodes; i++)
    {
      i2cDevices[i].open(bus,i + firstAddress);
      pressures[i].resize(numPressuresPerNode);
      pressureCommands[i].resize(numPressuresPerNode);
    }
  
  for(int i=0; i<numNodes; i++)
    {
      std::string topicString = "/robo_" + std::to_string(i) + "/joint_" + std::to_string(i) + "/pressure_command";
     /*
      See https://answers.ros.org/question/63991/how-to-make-callback-function-called-by-several-subscriber/?answer=63998?answer=63998#post-id-63998 for more details on this trickery.
     */
      ros::Subscriber sub = n.subscribe<sensor_msgs::FluidPressure>(topicString, 1000, boost::bind(&PressureController::pcmd_callback, this, _1, i));
      pressureCommandSubscribers.push_back(sub);
      ROS_INFO("/pressure_command topic started for joint %d",i);
    }

  for(int i=0; i<numNodes; i++)
    {
      std::string topic_string = "/robo_" + std::to_string(i) + "/joint_" + std::to_string(i) + "/pressure_state";
      ros::Publisher pub = n.advertise<sensor_msgs::FluidPressure>(topic_string,1000);
      pressurePublishers.push_back(pub);
      ROS_INFO("/pressure_state topic started for joint %d", i);
    }
}

int PressureController::get_num_devices_on_bus(int bus, int firstAddress)
{
  ROS_INFO_STREAM("Scanning for i2c devices on bus " << bus << "...");
  int numDevices = 0;
  bool done = false;
  while(!done)
    {
      I2CDevice device;
      int addr = numDevices + firstAddress;
      unsigned char testchar;

      device.open(bus,addr);
      bool error = device.readRegisters(0,sizeof(testchar),&testchar);
      
      if(!error)
	{
	  numDevices++;
	  ROS_INFO_STREAM("Found device 0x" << std::hex << addr << " on bus " << bus);
	}
      else
	{
	  done = true;
	}
    }

  ROS_INFO_STREAM("Found " << numDevices << " devices on bus " << bus);
  return numDevices;
}


void PressureController::do_pressure_control()
{
  while(ros::ok())
    {
      
      ROS_INFO_STREAM_ONCE("PRESSURE CONTROL STARTED");

      // update memory values for  pressure commands and pressures
      for(int node=0; node<numNodes; node++)
	{
	  unsigned char pchar[numPressuresPerNode*2];
	  for(int p=0; p<numPressuresPerNode; p++)
	    {
	      float_to_two_bytes(pressureCommands[node][p],&pchar[p*2]);
	    }

	  i2cDevices[node].writeRegisters(0,sizeof(pchar),&pchar[0]);

	  bool error = i2cDevices[node].readRegisters(0,sizeof(pchar),&pchar[0]);

	  if(error){
	    // send error message over ROS once per second on i2c failure
	    ROS_ERROR_STREAM_THROTTLE(1,"I2C Failure on node " << node);
	  }

	  for(int p=0; p<numPressuresPerNode; p++){
	      pressures[node][p] = two_bytes_to_float(&pchar[p*2]);
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
      for(int node = 0; node < numNodes; node++)
	{
	  sensor_msgs::FluidPressure msg;
	  msg.header = std_msgs::Header();
	  msg.header.stamp = ros::Time::now();

	  msg.pressure.resize(numPressuresPerNode);
	  
	  for(int p=0; p<numPressuresPerNode; p++)
	    {
	      msg.fluid_pressure[p] = pressures[node][p];
	    }
	  pressurePublishers[node].publish(msg);
	}
	
      ros::spinOnce();

    }

  //outside of while loop
  std::cout << "outside of loop" << std::endl;
  
  //disable power to arduinos
  estopOut.setValue(GPIO::LOW);

}


float PressureController::two_bytes_to_float(unsigned char * twobytes)
{
  uint16_t myint;
  memcpy(&myint, twobytes, 2);
  //std::cout <<  "myint: " << myint << std::endl;
  float myfloat = (float)(100.0*myint*psi2kpa/65535.0);
  //std::cout << "Pressure: " << myfloat << std::endl;
  return myfloat;
}

void PressureController::float_to_two_bytes(float myfloat, unsigned char * twobytes)
{
  //std::cout << "Pressure command: " << myfloat << std::endl;
  uint16_t myint = (myfloat/(100.0*psi2kpa)) * 65535.0;
  memcpy(twobytes, &myint, 2);
}

void PressureController::pcmd_callback(const sensor_msgs::FluidPressure::ConstPtr& msg, int node)
{
  for(int i=0; i < msg->data.size(); i++)
    {
      pressureCommands[node][i] = msg->fluid_pressure[i];
    }
}
