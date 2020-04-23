#include<sstream>
#include "PressureController.h"
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>

int PressureController::get_num_devices_on_bus(int bus, int firstAddress)
{
  int numDevices = 0;
  bool done = false;
  while(!done)
    {
      I2CDevice::I2CDevice() device;
      int addr = numDevices + firstAddress;
      int error = device.open(bus,addr);
      if(!error)
	{
	  numDevices++;
	}
      else
	{
	  done = true;
	}
    }
  return numDevices;
}

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
  
  ros::Publisher p_pub = n.advertise<std_msgs::Float32MultiArray>("pressures", 1000);

  ros::Subscriber pcmd_sub = n.subscribe("pressure_commands", 1000, pcmd_callback);  
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
	      pressures[p] = two_bytes_to_float(&pressureChars[node][p*2]);
	    }
	  
	}
    }

  std_msgs::Float32MultiArray msg;
  msg.data.resize(numNodes*numPressuresPerNode);
  for(int node = 0; node < numNodes; node++)
    {
      for(int p=0; p<numPressuresPerNode; p++)
	{
	  msg.data[node*numPressuresPerNode] = pressures[node][p];
	}
    }
      
  p_pub.publish(msg);
  ros::spinOnce();
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

void PressureController::pcmd_callback(const std_msgs::Float32MultiArray msg)
{
  for(int i=0; i<msg.data.size(); i++)
    {
      pressure_commands[i] = msg.data[i];
    }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "PressureController");
  ros::NodeHandle n;
  ros::Publisher p_pub = n.advertise<std_msgs::Float32MultiArray>("pressures", 1000);
  ros::Subscriber pcmd_sub = n.subscribe("pressure_commands", 1000, pcmd_callback);

  int i2c_bus = 1;
  I2CDevice device1(i2c_bus,11);
  // I2CDevice device2(i2c_bus,12);
  // I2CDevice device3(i2c_bus,13);
  

  // float p2[4] = {0,0,0,0};
  // float pcmd2[4] = {12.34, 56.78, 20.78, 46.82};
  // unsigned char p2char[4*2];    
  // unsigned char pcmd2char[4*2];

  // float p3[4] = {0,0,0,0};
  // float pcmd3[4] = {15.0,56.78, 20.78, 46.82};
  // unsigned char p3char[4*2];    
  // unsigned char pcmd3char[4*2];
  
  while(ros::ok())
    {
      for(int i=0; i<4; i++)
	{
	  float_to_two_bytes(pcmd1[i],&pcmd1char[i*2]);
	  // float_to_two_bytes(pcmd2[i],&pcmd2char[i*2]);
	  // float_to_two_bytes(pcmd3[i],&pcmd3char[i*2]);	  
	}
      
      device1.writeRegisters(0,sizeof(pcmd1char),&pcmd1char[0]);
      // device2.writeRegisters(0,sizeof(pcmd2char),&pcmd2char[0]);
      // device3.writeRegisters(0,sizeof(pcmd3char),&pcmd3char[0]);
      
      device1.readRegisters(0,sizeof(p1char),&p1char[0]);
      // device2.readRegisters(0,sizeof(p2char),&p2char[0]);
      // device3.readRegisters(0,sizeof(p3char),&p3char[0]);
      
      for(int i=0; i<4; i++)
	{
	  p1[i] = two_bytes_to_float(&p1char[i*2]);
	  // p2[i] = two_bytes_to_float(&p2char[i*2]);
	  // p3[i] = two_bytes_to_float(&p3char[i*2]);	  
	}
      
      std_msgs::Float32MultiArray msg;
      msg.data = {p1[0], p1[1], p1[2], p1[3]};
      // msg.data = {0,0,0,0, p2[0], p2[1], p2[2], p2[3], p3[0], p3[1], p3[2], p3[3]};
      // std::cout<<"pcmd: "<<pcmd1[0]<<"  "<<pcmd1[1]<<"  "<<pcmd1[2]<<"  "<<pcmd1[3]<<std::endl;
      
      p_pub.publish(msg);
      ros::spinOnce();
      // rate.sleep();
    }
  return 0;  
}
