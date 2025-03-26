#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include <chrono>
#include "PressureController.hpp"
#include <unistd.h>
#include <thread>

#include <wiringPi.h>
#include <wiringSerial.h>

PressureController::PressureController(int num_joints)
  : Node("pressure_controller")
{
  this->rs485_addresses["joint_0"] = 0xFFFF;
  if (num_joints > 1) {
    this->rs485_addresses["joint_1"] = 0xFFFE;
  }
  if (num_joints > 2) {
    this->rs485_addresses["joint_2"] = 0xFFFD;
  }

  this->initializeSerial();
  this->initializeDataVectors();
  this->ping_devices();

  // Getting the parameter rs485_config
  // rs485_addresses = this->convert_parameter_map(this->get_parameter("rs485_config"));

  // ######### Commented out by Curtis. Why do we need multithreaded operations launched here? ########
  // Add this node to the executor
  //executor.add_node(this->get_node_base_interface());

  // Start the executor in a separate thread
  //std::thread executor_thread([this, node]() {
  //    RCLCPP_INFO(this->get_logger(), "Starting executor");
  //    executor.spin();  // Spin the executor to process callbacks
  //});
  //executor_thread.detach();  // Detach the thread to run asynchronously

  this->startSubscribers();
  this->startPublishers();

  control_thread = std::thread([this]() {this->serial_loop();});
  control_thread.detach();
}

PressureController::~PressureController()
{
  serialFlush(this->fd);
  serialClose(this->fd);
}

void PressureController::ping_devices()
{
  serialFlush(this->fd);
  RCLCPP_INFO(this->get_logger(), "Checking communication with serial devices...");

  for (int joint = 0; joint < numJoints; joint++)
  {
    unsigned short jointAddress = this->rs485_addresses["joint_" + std::to_string(joint)];
    RCLCPP_INFO(this->get_logger(), "Pinging joint %d", jointAddress);

    prepareOutgoingBytes(joint);

    ssize_t numBytesWritten = write(this->fd, this->outgoingBytes, BYTES_IN_PACKET);

    bool timeout = waitForResponse(2000);

    if (!timeout)
    {
      if (handleIncomingBytes(joint))
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Joint " << jointAddress << " ping successful.");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unsucessful read");
        //RCLCPP_ERROR_ONCE(this->get_logger(), "Not all devices found. Killing node.");
        //rclcpp::shutdown();
	throw std::runtime_error("Communication attempted and failed. Killing node.");
      }
    }
    else
    {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Not all devices found. Killing node.");
      //rclcpp::shutdown();
      throw std::runtime_error("Timeout. Killing node.");
    }
  }
}

void PressureController::serial_loop()
{
  rclcpp::Duration max_loop_time(0, 0);
  unsigned long numLoops = 0;
  unsigned long numCorrupted = 0;
  unsigned long numTimeout = 0;

  while (rclcpp::ok())
  {
    rclcpp::Time loop_start = this->get_clock()->now(); 
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "PRESSURE CONTROL STARTED");

    for (int joint = 0; joint < numJoints; joint++)
    {
      if (DEBUG_MODE)
      {
        // rclcpp::Duration(.03, 0).sleep();
      }

      serialFlush(fd); // clear the current serial buffer before doing anything with it, RX AND TX

      prepareOutgoingBytes(joint);

      if (DEBUG_MODE)
      {
        printf("Pressure commands: ");
	this->mutex_lock.lock();
        for (int i = 0; i < 4; i++)
        {
          printf("%f ", this->pressureCommands[joint][i]);
        }
        printf("\n");
	this->mutex_lock.unlock();

        printf("Address + outgoing pressure command shorts: ");
        for (int i = 0; i < 5; i++)
        {
          printf("%d ", this->outgoingShorts[i]);
        }
        printf("\n");

        printf("Address + outgoing pressure command bytes: ");
        for (int i = 0; i < 10; i++)
        {
          printf("%02x ", this->outgoingBytes[i]);
        }
        printf("\n");
      }

      if (write(this->fd, this->outgoingBytes, BYTES_IN_PACKET) != 10)
      {
        RCLCPP_WARN(this->get_logger(), "Incorrect amount of bytes sent.");
      }

      bool timeout = waitForResponse(2);

      if (!timeout)
      {
        if (handleIncomingBytes(joint))
        {
          // convert analog shorts to kpa and load for sending over ROS
	  this->mutex_lock.lock();
          for (size_t i = 0; i < numPressuresPerJoint; i++)
          {
            float tmp = analogToKpa(this->incomingDataShorts[i]);
            this->pressures[joint][i] = filter(this->pressures[joint][i], tmp);
          }
	  this->mutex_lock.unlock();

          if (DEBUG_MODE)
          {
            printf("Converted to pressures: ");
	    this->mutex_lock.lock();
            for (int i = 0; i < 4; i++)
            {
              printf("%f ", this->pressures[joint][i]);
            }
            printf("\n");
	    this->mutex_lock.unlock();
          }

          // reset counters since communication was succesful
          jointMissedCounter[joint] = 0;
        }
        else
        {
          // printf("Unsucessful read. Data not saved.\n");
          numCorrupted++;
          jointMissedCounter[joint]++;
        }
      }
      else
      {
        numTimeout++;
        jointMissedCounter[joint]++;

        // empty both RX and TX buffers
        serialFlush(fd);
      }

      //check if we have lots of consecutive misses, likely something broke
      if (jointMissedCounter[joint] > 50)
      {
        // RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), this->get_clock(), 1, "Lost connection with joint " << joint);
      }
    }

    rclcpp::Duration loop_time = this->get_clock()->now() - loop_start;
    if (loop_time > max_loop_time)
    {
      max_loop_time = loop_time;
    }

    numLoops++;

    if (DEBUG_MODE)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Loop Time: " << this->get_clock()->now().seconds() - loop_start.seconds() << " s");
      std::cout << std::endl;
    }
  }

  // print serial communication statistics
  std::cout << "\n\nSERIAL COMMUNICATION STATISTICS\n" << std::endl;
  std::cout << "Max Loop Time: " << max_loop_time.seconds() << " s" << std::endl;
  std::cout << "Corrupted " << float(numCorrupted) / numLoops * 100 << "% of messages" << std::endl;
  std::cout << "Timed out " << float(numTimeout) / numLoops * 100 << "% of messages\n\n" << std::endl;
}

void PressureController::publishCallback()
{
  // publish pressures
  for (int joint = 0; joint < numJoints; joint++)
  {
    rad_msgs::msg::PressureStamped msg;
    msg.header = std_msgs::msg::Header();
    msg.header.stamp = this->get_clock()->now();

    msg.pressure.resize(numPressuresPerJoint);

    this->mutex_lock.lock();
    for (int p = 0; p < numPressuresPerJoint; p++)
    {
      msg.pressure[p] = pressures[joint][p];
    }
    this->mutex_lock.unlock();
    pressurePublishers[joint]->publish(msg);
  }
}

void PressureController::shortToBytes(unsigned short* short_array, unsigned char* byte_array)
{
  // Function to convert array of 5 shorts to array of 10 bytes
  int shortLength = 5;
  for (int i = 0; i < shortLength; i++)
  {
    int byteIndex = i * 2;
    unsigned char* bytePtr = (unsigned char*)&short_array[i];

    // convert from big endian to little endian by switching bytes around
    byte_array[byteIndex] = bytePtr[1];     // LSB
    byte_array[byteIndex + 1] = bytePtr[0]; // MSB
  }
}

void PressureController::byteToShorts(unsigned short* short_array, unsigned char* byte_array)
{
  // Function to convert array of 10 bytes to array of 5 shorts
  unsigned int byteLength = 10;
  for (size_t i = 0; i < byteLength; i += 2)
  {
    short_array[i / 2] = ((short)byte_array[i] << 8) | byte_array[i + 1];
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

void PressureController::startSubscribers()
{
  std::string ns = this->get_name(); 
  // Create pressure command subscribers
  for (int i = 0; i < numJoints; i++)
  {
    std::string topicString = ns + "/joint_" + std::to_string(i) + "/pressure_command";
    /*
      See https://answers.ros.org/question/63991/how-to-make-callback-function-called-by-several-subscriber/?answer=63998?answer=63998#post-id-63998 for more details on this trickery.
     */
    rclcpp::Subscription<rad_msgs::msg::PressureStamped>::SharedPtr sub = this->create_subscription<rad_msgs::msg::PressureStamped>(topicString, 1, [this, i](const rad_msgs::msg::PressureStamped::SharedPtr msg) {this->pcmd_callback(msg, i);}); 
    pressureCommandSubscribers.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "/pressure_command topic started for joint %d", i);
  }
}

void PressureController::startPublishers()
{
  std::string ns = this->get_name();
  // Create pressure data publisheers
  for (int i = 0; i < numJoints; i++)
  {
    std::string topic_string = ns + "/joint_" + std::to_string(i) + "/pressure_state";
    rclcpp::Publisher<rad_msgs::msg::PressureStamped>::SharedPtr pub = this->create_publisher<rad_msgs::msg::PressureStamped>(topic_string, 1);
    pressurePublishers.push_back(pub);
    RCLCPP_INFO(this->get_logger(), "/pressure_state topic started for joint %d", i);
  }

  this->publisher_timer = this->create_wall_timer(std::chrono::milliseconds(2), [this]() {this->publishCallback(); });
}

void PressureController::pcmd_callback(const rad_msgs::msg::PressureStamped::SharedPtr msg, int joint)
{
  this->mutex_lock.lock();
  for (int i = 0; i < msg->pressure.size(); i++)
  {
    float temp = (float)msg->pressure[i]; // cast double/float64 to float/float to send over i2c
    pressureCommands[joint][i] = temp;
  }
  this->mutex_lock.unlock();
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
    if (DEBUG_MODE)
    {
      std::cout << "Bytes available on handle: " << std::dec << serialDataAvail(fd) << std::endl;
    }

    unsigned char secondByte = serialGetchar(fd);

    if (DEBUG_MODE)
    {
      printf("First Byte: %02x", firstByte);
      printf(" Second Byte: %02x\n", secondByte);
    }

    unsigned short short1 = (firstByte << 8) | secondByte;

    if (DEBUG_MODE)
    {
      printf("short to check: %d\n", short1);
    }

    if (short1 == jointAddress)
    {
      if (read(this->fd, this->incomingDataBytes, BYTES_IN_PACKET - 2) == BYTES_IN_PACKET - 2)
      {
        byteToShorts(this->incomingDataShorts, this->incomingDataBytes);

        if (DEBUG_MODE)
        {
          std::cout << "Address match!" << std::endl;
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
        for (int i = 0; i < 4; i++)
        {
          // must be limited by 10 bit resolution of ADC, otherwise this is incorrect
          if (this->incomingDataShorts[i] > 1023)
          {
            readSuccessful = false;
            // printf("Invalid shorts recieved. Should be 0-1023. Throwing away data.\n");
            break;
          }
          else
          {
            readSuccessful = true;
          }
        }
        firstByte = 0;
      }
    }
    else
    {
      if (DEBUG_MODE)
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

  // fill appropriate commands from this->pressure_commands into outgoing Shorts
  for (int i = 0; i < 4; i++)
  {
    this->outgoingShorts[i + 1] = kpaToAnalog(this->pressureCommands[joint][i]);
  }

  shortToBytes(this->outgoingShorts, this->outgoingBytes); // converts test_short_write (shorts) to check_msg_write (bytes)
}

float PressureController::filter(float prev, float input)
{ /*
     This function implements a first order low pass filter
     with a cutoff frequency of 50 Hz. First order hold discrete implementation with dt=.001.
     First order filter is of form:
     a / (z - b)
 */
  float a = 0.6;
  float b = 1.0 - a;

  return b * prev + a * input;
}
