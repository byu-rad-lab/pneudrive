#include <SoftwareWire.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

#define    MPU9250_ADDRESS            0x69
#define    MAG_ADDRESS                0x0C
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    ACC_FULL_SCALE_2_G        0x00  

Madgwick filter;
float rpy[3] = {0, 0, 0};
char rpychar[2*3];

bool visualize = true;
bool calibrate_acc = false;
bool calibrate_mag = false;
bool calibrate_gyr = false;

float ax_min = 1000;
float ay_min = 1000;
float az_min = 1000;
float ax_max = -1000;
float ay_max = -1000;
float az_max = -1000;
float mx_min = 1000;
float my_min = 1000;
float mz_min = 1000;
float mx_max = -1000;
float my_max = -1000;
float mz_max = -1000;
float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;

SoftwareWire softWire(11,10);

int i2c_address = 101;

void setup()
{
  Serial.begin(2000000);

  Wire.begin(i2c_address);
  Wire.onRequest(requestEvent);
  
  softWire.begin();
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

  for(int i=0; i<5; i++)
  {
    digitalWrite(13,!digitalRead(13));
    delay(100);
  }
  for(int i=0; i<10; i++)
  {
    digitalWrite(13,!digitalRead(13));
    delay(50);
  }  
}

void loop()
{
  unsigned long tic = micros();

  // Read user input
  if (Serial.available() >= 2)
  {
    if (Serial.read() == '#') // Start of new control message
    {
      int command = Serial.read(); // Commands
      if (command == 'v') // visualize
      {
        visualize = true;
        calibrate_mag = false;
        calibrate_acc = false;
        calibrate_gyr = false;
      }
      else if (command == 'm')
      {
        visualize = false;
        calibrate_mag = true;
        calibrate_acc = false;
        calibrate_gyr = false;
      }
      else if (command == 'a')
      {
        visualize = false;
        calibrate_mag = false;
        calibrate_acc = true;
        calibrate_gyr = false;
      }
      else if (command == 'g')
      {
        visualize = false;
        calibrate_mag = false;
        calibrate_acc = false;
        calibrate_gyr = true;
      }
    }
  }
  
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data
  // Accelerometer
  int16_t aix=(Buf[0]<<8 | Buf[1]);
  int16_t aiy=(Buf[2]<<8 | Buf[3]);
  int16_t aiz=(Buf[4]<<8 | Buf[5]);

  // Gyroscope
  int16_t gix=(Buf[8]<<8 | Buf[9]);
  int16_t giy=(Buf[10]<<8 | Buf[11]);
  int16_t giz=(Buf[12]<<8 | Buf[13]);

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  
  // Create 16 bits values from 8 bits data
  // Magnetometer
  int16_t mix=(Mag[1]<<8 | Mag[0]);
  int16_t miy=(Mag[3]<<8 | Mag[2]);
  int16_t miz=(Mag[5]<<8 | Mag[4]);

  if(calibrate_acc==true)
  {
    if(aix>ax_max){ax_max=aix;}
    if(aiy>ay_max){ay_max=aiy;}
    if(aiz>az_max){az_max=aiz;}
    if(aix<ax_min){ax_min=aix;}
    if(aiy<ay_min){ay_min=aiy;}
    if(aiz<az_min){az_min=aiz;}
    Serial.print("ax_min: ");
    Serial.print(ax_min);
    Serial.print("  ax_max: ");
    Serial.print(ax_max);
    Serial.print("  ay_min: ");
    Serial.print(ay_min);
    Serial.print("  ay_max: ");
    Serial.print(ay_max);
    Serial.print("  az_min: ");
    Serial.print(az_min);
    Serial.print("  az_max: ");
    Serial.println(az_max);
  }
  if(calibrate_mag==true)
  {
    if(mix>mx_max){mx_max=mix;}
    if(miy>my_max){my_max=miy;}
    if(miz>mz_max){mz_max=miz;}
    if(mix<mx_min){mx_min=mix;}
    if(miy<my_min){my_min=miy;}
    if(miz<mz_min){mz_min=miz;}
    Serial.print("mx_min: ");
    Serial.print(mx_min);
    Serial.print("  mx_max: ");
    Serial.print(mx_max);
    Serial.print("  my_min: ");
    Serial.print(my_min);
    Serial.print("  my_max: ");
    Serial.print(my_max);
    Serial.print("  mz_min: ");
    Serial.print(mz_min);
    Serial.print("  mz_max: ");
    Serial.println(mz_max); 
  }
  if(calibrate_gyr==true)
  {
    gx_offset = .999*gx_offset + .001*float(gix);
    gy_offset = .999*gy_offset + .001*float(giy);
    gz_offset = .999*gz_offset + .001*float(giz);
    Serial.print("gx_offset: ");
    Serial.print(gx_offset);
    Serial.print("  gy_offset: ");
    Serial.print(gy_offset);
    Serial.print("  gz_offset: ");
    Serial.println(gz_offset);
  }

  // Update calibration
  float ax_range = ax_max-ax_min;
  float ax_mid = .5*(ax_max+ax_min);
  float ay_range = ay_max-ay_min;
  float ay_mid = .5*(ay_max+ay_min);
  float az_range = az_max-az_min;
  float az_mid = .5*(az_max+az_min);
  float mx_range = mx_max-mx_min;
  float mx_mid = .5*(mx_max+mx_min);
  float my_range = my_max-my_min;
  float my_mid = .5*(ay_max+my_min);
  float mz_range = mz_max-mz_min;
  float mz_mid = .5*(mz_max+mz_min);
 
  // Apply calibration bias and gain
  // Also - magnetometer and accel/gyro measurements are not in the same frame
  // Also - I negate the accel vector so it points down instead of up
  float ax = -convertRawAcceleration((aiy-ay_mid)*2.0/ay_range);
  float ay = -convertRawAcceleration((aix-ax_mid)*2.0/ax_range);
  float az = convertRawAcceleration((aiz-az_mid)*2.0/az_range);
  float gx = convertRawGyro(giy-gy_offset);
  float gy = convertRawGyro(gix-gx_offset);
  float gz = -convertRawGyro(giz-gz_offset);
  float mx = (mix-mx_mid)*2/mx_range;
  float my = (miy-my_mid)*2/my_range;
  float mz = (miz-mz_mid)*2/mz_range;

  

  if(visualize==true)
  {
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    rpy[0] = filter.getRoll();
    rpy[1] = filter.getPitch();
    rpy[2] = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(rpy[0]);
    Serial.print(" ");
    Serial.print(rpy[1]);
    Serial.print(" ");
    Serial.println(rpy[2]);
  }
  filter.invSampleFreq = float(micros()-tic)/1000000.0;
}

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  softWire.beginTransmission(Address);
  softWire.write(Register);
  softWire.endTransmission();
  
  // Read Nbytes
  softWire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (softWire.available())
    Data[index++]=softWire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  softWire.beginTransmission(Address);
  softWire.write(Register);
  softWire.write(Data);
  softWire.endTransmission();
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void requestEvent()
{ 
  Wire.write(rpychar, sizeof(rpychar));
}

void float_to_two_bytes(float myfloat, unsigned char * twobytes)
{
  uint16_t myint = myfloat * 65536.0 / 720.0;
  memcpy(twobytes, &myint, 2);
}
