#include<sstream>
#include "PressureController.h"


PressureController::PressureController(int bus=1, int firstAddress=10, int pressuresPerNode=4)
{
  numNodes = get_num_devices_on_bus(bus,firstAddress);
  numPressuresPerNode = pressuresPerNode;
  
  i2cDevices.resize(numNodes);
  pressures.resize(numNodes);
  pressureCommands.resize(numNodes);
  pressureChars.resize(numNodes);
  pressureCommandChars.resize(numNodes);
  
  for(int i=0; i<numNodes; i++)
    {
      i2cDevices[i].open(bus,i + firstAddress);
      pressures[i].resize(numPressuresPerNode);
      pressureCommands[i].resize(numPressuresPerNode);
      pressureChars[i].resize(numPressuresPerNode*2); //two bytes per pressure
      pressureCommandChars[i].resize(numPressuresPerNode*2); //two bytes per pressure
    }
  
  int argc;
  char **argv;
  ros::init(argc, argv, "PressureController");
  ros::NodeHandle n;

  for(int i=0; i<numNodes; i++)
    {
      std::string topicString = "/node_" + std::to_string(i) + "/pressure_commands";
      ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>(topicString, 1000, boost::bind(&PressureController::pcmd_callback, this, _1, i));
      pressureCommandSubscribers.push_back(sub);
    }

  for(int i=0; i<numNodes; i++)
    {
      std::string topic_string = "/node_" + std::to_string(i) + "/pressures";
      ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>(topic_string,1000);
      pressurePublishers.push_back(pub);
    }
  
}

int PressureController::get_num_devices_on_bus(int bus, int firstAddress)
{
  std::cout<<"Looking for devices on the bus!"<<std::endl;  
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
	}
      else
	{
	  done = true;
	}
    }
  std::cout<<"I found "<<numDevices<<" devices on the bus!"<<std::endl;
  return numDevices;
}


void PressureController::do_pressure_control()
{
  while(ros::ok())
    {
      for(int node=0; node<numNodes; node++)
	{
	  for(int byte=0; byte<numPressuresPerNode; byte++)
	    {
	      float_to_two_bytes(pressureCommands[node][byte],&pressureCommandChars[node][byte*2]);
	    }
	  
	  i2cDevices[node].writeRegisters(0,sizeof(pressureCommandChars[node]),&pressureCommandChars[node][0]);

	  i2cDevices[node].readRegisters(0,sizeof(pressureChars),&pressureChars[node][0]);

	  for(int p=0; p<numPressuresPerNode; p++)
	    {
	      pressures[node][p] = two_bytes_to_float(&pressureChars[node][p*2]);
	    }
	  
	}

      for(int node = 0; node < numNodes; node++)
	{
	  std_msgs::Float32MultiArray msg;
	  msg.data.resize(numPressuresPerNode);
	  
	  for(int p=0; p<numPressuresPerNode; p++)
	    {
	      msg.data[p] = pressures[node][p];
	    }
	  pressurePublishers[node].publish(msg);
	}
      
      ros::spinOnce();
    }
}


float PressureController::two_bytes_to_float(unsigned char * twobytes)
{
  uint16_t myint;
  memcpy(&myint, twobytes, 2);
  float myfloat = (float)(100.0*myint/65536.0);
  return myfloat;
}

void PressureController::float_to_two_bytes(float myfloat, unsigned char * twobytes)
{
  uint16_t myint = myfloat*65536.0/100.0;
  memcpy(twobytes, &myint, 2);
}

void PressureController::pcmd_callback(const std_msgs::Float32MultiArray::ConstPtr& msg, int node)
{
  for(int i=0; i<msg->data.size(); i++)
    {
      pressureCommands[node][i] = msg->data[i];
    }
}
