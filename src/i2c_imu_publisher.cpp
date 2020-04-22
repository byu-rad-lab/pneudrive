#include<sstream>
#include<iostream>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include "I2CDevice.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "IMUPublisher");
  ros::NodeHandle n;
  
  // ros::Publisher p_pub = n.advertise<std_msgs::Float32MultiArray>("imu_rpy", 1000);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  
  // ros::Rate rate(50); //Hz

  int i2c_bus = 2;
  // I2CDevice device1(i2c_bus,11);
  I2CDevice device2(i2c_bus,12);
  // I2CDevice device3(i2c_bus,13);

  float rpy1[3] = {0,0,0};
  float rpy2[3] = {0,0,0};
  float rpy3[3] = {0,0,0};

  unsigned char rpychar1[3*4]; //3 floats of 4 bytes each
  unsigned char rpychar2[3*4]; //3 floats of 4 bytes each
  unsigned char rpychar3[3*4]; //3 floats of 4 bytes each

  
  
  while(ros::ok())
    {
      // device1.readRegisters(0,sizeof(rpychar1),&rpychar1[0]);
      device2.readRegisters(0,sizeof(rpychar2),&rpychar2[0]);      
      // device3.readRegisters(0,sizeof(rpychar3),&rpychar3[0]);

      // for loop over floats from arduino (roll,pitch,yaw)
      for(int i=0; i<3; i++)
	{
	  // memcpy(&rpy1[i], &rpychar1[i*4], 4);
	  memcpy(&rpy2[i], &rpychar2[i*4], 4);
	  // memcpy(&rpy3[i], &rpychar3[i*4], 4);	  
	}

      // For publishing with tf
      // transform.setOrigin( tf::Vector3(0,0,0) );
      // q.setRPY(rpy1[0]*3.14159/180.0,rpy1[1]*3.14159/180.0,rpy1[2]*3.14159/180.0);
      // transform.setRotation(q);
      // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "IMU 1"));

      transform.setOrigin( tf::Vector3(1,0,0) );
      q.setRPY(rpy2[0]*3.14159/180.0,rpy2[1]*3.14159/180.0,rpy2[2]*3.14159/180.0);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "IMU 2"));
      
      // For publishing as floats
      // std_msgs::Float32MultiArray msg;
      // msg.data = {rpy1[0], rpy1[1], rpy1[2], rpy2[0], rpy2[1], rpy2[2], rpy3[0], rpy3[1], rpy3[2]};
      // p_pub.publish(msg);
      
      // std::cout<<rpy1[0]<<"  "<<rpy1[1]<<"  "<<rpy1[2]<<"  "<<rpy2[0]<<"  "<<rpy2[1]<<"  "<<rpy2[2]<<"  "<<rpy3[0]<<"  "<<rpy3[1]<<"  "<<rpy3[2]<<std::endl;
      
      ros::spinOnce();
      // rate.sleep();
    }
  return 0;  
}
