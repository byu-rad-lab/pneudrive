#include<sstream>
#include <tf/transform_broadcaster.h>
#include "IMUPublisher.h"


IMUPublisher::IMUPublisher(int bus=2, int firstAddress=100)
{
  numNodes = get_num_devices_on_bus(bus,firstAddress);
  
  i2cDevices.resize(numNodes);
  poses.resize(numNodes);
  
  for(int i=0; i<numNodes; i++)
    {
      i2cDevices[i].open(bus,i + firstAddress);
      poses[i].resize(3); // 3 for RPY
    }
  
  int argc;
  char **argv;
  ros::init(argc, argv, "IMUPublisher");
  ros::NodeHandle n;
}

int IMUPublisher::get_num_devices_on_bus(int bus, int firstAddress)
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


void IMUPublisher::run()
{
  while(ros::ok())
    {
      for(int node=0; node<numNodes; node++)
	{
	  unsigned char rpychar[3*2];
	  i2cDevices[node].readRegisters(0,sizeof(rpychar),&rpychar[0]);
	  for(int i=0; i<3; i++)
	    {
	      poses[node][i] = two_bytes_to_float(&rpychar[i*2]);
	    }
	}

      for(int node = 0; node < numNodes; node++)
	{
	  static tf::TransformBroadcaster br;
	  tf::Transform transform;
	  tf::Quaternion q;
	  transform.setOrigin( tf::Vector3(1,0,0) );
	  q.setRPY(poses[node][0]*3.14159/180.0,poses[node][1]*3.14159/180.0,poses[node][2]*3.14159/180.0);
	  transform.setRotation(q);
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "IMU "+std::to_string(node)));
	}
      
      ros::spinOnce();
    }
}

float IMUPublisher::two_bytes_to_float(unsigned char * twobytes)
{
  uint16_t myint;
  memcpy(&myint, twobytes, 2);
  float myfloat = (float)(720.0*myint/65536.0);
  return myfloat;
}

void IMUPublisher::float_to_two_bytes(float myfloat, unsigned char * twobytes)
{
  uint16_t myint = myfloat*65536.0/720.0;
  memcpy(twobytes, &myint, 2);
}
