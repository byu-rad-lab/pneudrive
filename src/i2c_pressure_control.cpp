#include<sstream>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include "I2CDevice.h"

float two_bytes_to_float(unsigned char * twobytes)
{
  uint16_t myint;
  memcpy(&myint, twobytes, 2);
  float myfloat = (float)(100.0*myint/65536.0);
  return myfloat;
}

void float_to_two_bytes(float myfloat, unsigned char * twobytes)
{
  uint16_t myint = myfloat*65536.0/100.0;
  memcpy(twobytes, &myint, 2);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "PressureController");
  ros::NodeHandle n;
  ros::Publisher p_pub = n.advertise<std_msgs::Float32MultiArray>("pressures", 1000);  

  int i2c_bus = 2;
  I2CDevice device1(i2c_bus,11);
  I2CDevice device2(i2c_bus,12);
  I2CDevice device3(i2c_bus,13);
  
  float p1[4] = {0,0,0,0};
  float pcmd1[4] = {12.34, 56.78, 20.78, 46.82};
  unsigned char p1char[4*2];    
  unsigned char pcmd1char[4*2];

  float p2[4] = {0,0,0,0};
  float pcmd2[4] = {12.34, 56.78, 20.78, 46.82};
  unsigned char p2char[4*2];    
  unsigned char pcmd2char[4*2];

  float p3[4] = {0,0,0,0};
  float pcmd3[4] = {12.34, 56.78, 20.78, 46.82};
  unsigned char p3char[4*2];    
  unsigned char pcmd3char[4*2];
  
  while(ros::ok())
    {
      for(int i=0; i<4; i++)
	{
	  float_to_two_bytes(pcmd1[i],&pcmd1char[i*2]);
	  float_to_two_bytes(pcmd2[i],&pcmd2char[i*2]);
	  float_to_two_bytes(pcmd3[i],&pcmd3char[i*2]);	  
	}
      
      device1.writeRegisters(0,sizeof(pcmd1char),&pcmd1char[0]);
      device2.writeRegisters(0,sizeof(pcmd2char),&pcmd2char[0]);
      device3.writeRegisters(0,sizeof(pcmd3char),&pcmd3char[0]);
      
      device1.readRegisters(0,sizeof(p1char),&p1char[0]);
      device2.readRegisters(0,sizeof(p2char),&p2char[0]);
      device3.readRegisters(0,sizeof(p3char),&p3char[0]);
      
      for(int i=0; i<4; i++)
	{
	  p1[i] = two_bytes_to_float(&p1char[i*2]);
	  p2[i] = two_bytes_to_float(&p2char[i*2]);
	  p3[i] = two_bytes_to_float(&p3char[i*2]);	  
	}
      
      std_msgs::Float32MultiArray msg;
      msg.data = {p1[0], p1[1], p1[2], p1[3], p2[0], p2[1], p2[2], p2[3], p3[0], p3[1], p3[2], p3[3]};
      
      p_pub.publish(msg);
      ros::spinOnce();
    }
  return 0;  
}
