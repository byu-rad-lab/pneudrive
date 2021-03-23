#include<sstream>
#include "PressureController.h"

// conversion factor from psi to kpa (ie kpa = psi * psi2kpa)
const double psi2kpa = 6.8947572932;

PressureController::PressureController(int bus=1, int firstAddress=10, int pressuresPerNode=4)
{
  //enable power to arduinos
  estopOut.setValue(GPIO::LOW);

  numNodes = get_num_devices_on_bus(bus,firstAddress);
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
  
  int argc;
  char **argv;
  ros::init(argc, argv, "PressureController");
  ros::NodeHandle n;

  for(int i=0; i<numNodes; i++)
    {
      std::string topicString = "/node_" + std::to_string(i) + "/pressure_commands";
      ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>(topicString, 1000, boost::bind(&PressureController::pcmd_callback, this, _1, i));
      pressureCommandSubscribers.push_back(sub);
      ROS_INFO("/pressure_commands topic started for joint %d",i);
    }

  for(int i=0; i<numNodes; i++)
    {
      std::string topic_string = "/node_" + std::to_string(i) + "/pressures";
      ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>(topic_string,1000);
      pressurePublishers.push_back(pub);
      ROS_INFO("/pressures topic started for joint %d", i);
    }
  
}

int PressureController::get_num_devices_on_bus(int bus, int firstAddress)
{
  std::cout << "Looking for devices on bus " << bus << "..." << std::endl;
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
          std::cout << "Found device " << addr << " on bus " << bus << std::endl;
	}
      else
	{
	  done = true;
	}
    }
  std::cout<<"I found "<<numDevices<<" devices on the bus "<< bus << std::endl;
  return numDevices;
}


void PressureController::do_pressure_control()
{
  ROS_INFO("STARTING PRESSURE CONTROL");
  while(ros::ok())
    {
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
	    ROS_ERROR("I2C Failure on node %i", node);
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
	  std_msgs::Float32MultiArray msg;
	  msg.data.resize(numPressuresPerNode);
	  
	  for(int p=0; p<numPressuresPerNode; p++)
	    {
	      msg.data[p] = pressures[node][p] /psi2kpa;
	    }
	  pressurePublishers[node].publish(msg);
	}
	
      ros::spinOnce();

//      if (estopIn.getValue()==GPIO::HIGH){
//	ros::shutdown();
//     }else{
//        ros::spinOnce();
//     }
    }

  if(estopIn.getValue()==GPIO::HIGH){
    ROS_ERROR("Emergency Stop encountered. Disabling power.");
    estopOut.setValue(GPIO::HIGH);
  }
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

void PressureController::pcmd_callback(const std_msgs::Float32MultiArray::ConstPtr& msg, int node)
{
  std::cout << "pressure command recieved in callback function" << std::endl;
  for(int i=0; i < msg->data.size(); i++)
    {
      pressureCommands[node][i] = msg->data[i];
    }
}
