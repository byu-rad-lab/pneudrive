#include <Wire.h>

float p[4] = {0, 0, 0, 0};
float pcmd[4] = {0, 0, 0, 0};
float pbias[4] = {0, 0, 0, 0};

// arrays of 8 bytes used for i2c comms. Each pressure uses 2 bytes (16 bits).
byte pchar[2 * 4];
byte pcmdchar[2 * 4];

// pins used for driving valves
int digital_pins [4] = {8, 7, 4, 2};
int pwm_pins [4] = {9, 6, 5, 3};

int valve_cmd[4] = {0, 0, 0, 0}; //valve commands [0, 255]

//'Global' variables that are important for control
float myTime = 0.0;
float prevTime = 0.0; //Last time the loop was entered
float integrator[4] = {0, 0, 0, 0};
float pdot[4] = {0, 0, 0, 0};
float prevError[4] = {0, 0, 0, 0}; //error at the previous timestep
float prevP[4] = {0, 0, 0, 0};
float errorDot[4] = {.0, .0, .0, .0}; //Derivative of the error
float awl = 1.0; //Anti-Windup limit.
float kp = 10.0;
float ki = 0.000;
float kd = 0.00;
float sigma = .05; // Dirty Derivative Bandwidth = 1/sigma
float deadband = 0.0; // kpa, controller will not act on error less than deadband

float error = 0.0;
float dt = 0;

//float LP_COEFF[67] = {-0.0002852,0.0005758,0.0008177,0.001106,0.00122,0.0009859,0.0003053,-0.0007989,-0.002148,-0.003411,-0.004161,-0.003978,-0.002582,3.548e-05,0.003526,0.007179,0.01002,0.01102,0.009336,0.004626,-0.00275,-0.01163,-0.02014,-0.02595,-0.0267,-0.02049,-0.006382,0.0153,0.04285,0.07331,0.1029,0.1277,0.1442,0.1499,0.1442,0.1277,0.1029,0.07331,0.04285,0.0153,-0.006382,-0.02049,-0.0267,-0.02595,-0.02014,-0.01163,-0.00275,0.004626,0.009336,0.01102,0.01002,0.007179,0.003526,3.548e-05,-0.002582,-0.003978,-0.004161,-0.003411,-0.002148,-0.0007989,0.0003053,0.0009859,0.00122,0.001106,0.0008177,0.0005758,-0.0002852};
// motor driver error pins
const int EF1_A = 10;
const int EF2_A = 11;
const int EF1_B = 20;
const int EF2_B = 21;

// conversion factor from psi to kpa (ie kpa = psi * PSI2KPA)
double const PSI2KPA = 6.8947572932;
double const P_MAX = 100 * PSI2KPA;

void setup(void)
{
  Serial.begin(115200);

  // check i2c address
  int i2c_address;
  i2c_address = geti2caddress();
  Serial.print("I2C address: 0x");
  Serial.print(i2c_address, HEX);
  Serial.println();

  //set up I2C and register event handlers
  Wire.begin(i2c_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);


  // check data on pressure sensors
  float startTime = millis();
  float curTime = 0.0;
  Serial.print("Checking sensors for 30 seconds. Apply a known pressure to each sensor. Make sure following readings make sense.");
  Serial.println();
  delay(2000);
  while (curTime - startTime < 30 * 1000) {
    Serial.print(readPressure(A0) / PSI2KPA);
    Serial.print("\t");
    Serial.print(readPressure(A1) / PSI2KPA);
    Serial.print("\t");
    Serial.print(readPressure(A2) / PSI2KPA);
    Serial.print("\t");
    Serial.print(readPressure(A3) / PSI2KPA);
    Serial.println();
    curTime = millis();
  }

  //set up fault pins, input pullup bc no external pull is used.
  pinMode(EF1_A, INPUT_PULLUP);
  pinMode(EF2_A, INPUT_PULLUP);
  pinMode(EF1_B, INPUT_PULLUP);
  pinMode(EF2_B, INPUT_PULLUP);

  // check valve control connections
  Serial.print("Venting each valve control connection for 10 seconds. Use a multimeter to check voltages on each connector. Make sure 12V supply is connected");
  Serial.println();
  //set each digital/pwm pin as output and vent all valves
  for (int i = 0; i < 4; i++)
  {
    pinMode(digital_pins[i], OUTPUT);
    pinMode(pwm_pins[i], OUTPUT);
    move_valve(digital_pins[i], pwm_pins[i], -255);
  }

  delay(10000);
  
  Serial.print("Filling each valve control connection for 10 seconds. Use a multimeter to check voltages on each connector. Make sure 12V supply is connected");
  Serial.println();
  for (int i = 0; i < 4; i++)
  {
    move_valve(digital_pins[i], pwm_pins[i], 255);
  }

  delay(10000);
  
  Serial.print("Closing each valve control connection for 10 seconds. Use a multimeter to check voltages on each connector. Make sure 12V supply is connected");
  Serial.println();
  for (int i = 0; i < 4; i++)
  {
    move_valve(digital_pins[i], pwm_pins[i], 0);
  }
  
  //
  //  delay(5000);
  //  prevTime = millis();
}


void loop(void)
{

}


void move_valve(int digital_pin, int pwm_pin, int valve_cmd)
{
  /*
     Logic in this function is:
     if valve_cmd is positive -> measured pressure is below command -> fill
     if valve_cmd is negative -> measured pressure is above command -> vent

  */


  if (valve_cmd > 0)
  {
    //fill
    fill(digital_pin, pwm_pin, valve_cmd);
  }
  else if (valve_cmd < 0)
  {
    //vent
    vent(digital_pin, pwm_pin, valve_cmd);
  }else if (valve_cmd == 0)
  {
    close(digital_pin, pwm_pin, valve_cmd);
  }
}

void fill(int digital_pin, int pwm_pin, int valve_cmd)
{
  /*
     valve_cmd is positive, map valve_cmd to (0, 255].
     NOTE: this cooresponds to direction that the valves are plugged in.
     Looking at the top of the connector (where the metal is),
     the brown wire should be on the left, and the blue on the right.
  */



  if (valve_cmd > 255) {
    valve_cmd = 255;
  }

  digitalWrite(digital_pin, HIGH);
  analogWrite(pwm_pin, valve_cmd);
  //  if(digital_pin==7){
  //    Serial.print(valve_cmd);
  //    Serial.println();
  //   }
}

void vent(int digital_pin, int pwm_pin, int valve_cmd)
{
  /*
    valve_cmd is negative, map range to (0, 255].
    very negative means lots of effort, which driver board needs lots of effort to be low
    map [-255,0] --> [0,255]
  */

  if (valve_cmd < -255) {
    valve_cmd = -255;
  }
  digitalWrite(digital_pin, LOW);
  analogWrite(pwm_pin, valve_cmd + 255);
  //    if(digital_pin==7){
  //    Serial.print(valve_cmd);
  //    Serial.println();
  //   }
}

void close(int digital_pin, int pwm_pin, int valve_cmd)
{
    // brake on a4990 data sheet is when IN1/3 is opposite of IN2/4.
    digitalWrite(digital_pin, HIGH);
    analogWrite(pwm_pin, 0);
}

float dirtyDifferentiate(float input, float prev_input, float input_dot)
{
  /*
     This function takes a dirty derivative of input. BROKEN.
  */

  float beta = (2 * sigma - dt) / (2 * sigma + dt);

  return beta * input_dot + ((1 - beta) / dt) * (input - prev_input);

  //  return (input - prev_input)/(dt*.000001);
}

float filter(float prev, float input)
{
  /*
      This function implements a first order low pass filter
      with a cutoff frequency of 50 Hz. First order hold discrete implementation with dt=.001.
      First order filter is of form:
      a / (z - b)
  */

  float a = .2696;
  float b = .7304;

  return b * prev + a * input;

}

void receiveEvent(int howMany)
{
  if (howMany > 1)
  {
    //first byte of data stream is register to write to, I think
    Wire.read();
    //extract 8 bytes of pressure command (2 bytes/chamber)
    for (int j = 0; j < 4 * 2; j++)
    {
      pcmdchar[j] = Wire.read();
    }
  }
  else
  {
    Wire.read();
  }
}

void requestEvent()
{
  Wire.write(pchar, sizeof(pchar));
}

float two_bytes_to_float(byte * pcmdchar_twobytes)
{
  uint16_t myint;
  memcpy(&myint, pcmdchar_twobytes, 2);
  return float((100.0 * PSI2KPA) * myint / 65535.0);
}

void float_to_two_bytes(float myfloat, byte * pchar_twobytes)
{

  /*
     We have an issue here, sometimes the sensors
     read negative pressure b/c of the error band.
     We can calibrate, set limits in read pressure function, etc.
     Not sure what best thing to do is.
  */
  // convert float of kPa to bits to send over i2c
  // uint16_t can be 0 to 65535

  if (myfloat < 0) {
    myfloat = 0;
  }

  uint16_t myint = (myfloat / (100.0 * PSI2KPA)) * 65535.0;

  // memcpy(pointer to destination, pointer to source, # bytes to copy)
  // copy 2 bytes of data located @ myint to location of pchar_twobytes
  memcpy(pchar_twobytes, &myint, 2);
}

int geti2caddress()
{
  /* Addresses from dip switch
      a number indicates a HIGH value, _ is LOW
     PIN | Addr
     ---------
     _ _ = 0xA
     1 _ = 0xB
     1 2 = 0xC
     _ 2 = 0xD
  */

  int i2caddr;
  int one;
  int two;

  // b/c we are using pin 13 as an input, the LED_BUILTIN cannot be used.
  pinMode(LED_BUILTIN, INPUT);
  pinMode(12, INPUT);

  one = digitalRead(12);
  two = digitalRead(LED_BUILTIN);

  if (one == LOW && two == LOW) {
    i2caddr = 0xA;
  } else if (one == HIGH && two == LOW) {
    i2caddr = 0xB;
  } else if (one == HIGH && two == HIGH) {
    i2caddr = 0xC;
  } else if (one == LOW && two == HIGH) {
    i2caddr = 0xD;
  }

  //  Serial.print("I2C Address: ");
  //  Serial.print(i2caddr);
  //  Serial.println();

  return i2caddr;
}

double readPressure(int analogPin)
{
  /*
     ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
     Analog reference is 5V, so 0=0v and 1023=5v
     Conversion is taken from Fig. 3 (transfer function A) in pressure sensor datasheet
     https://www.mouser.com/datasheet/2/187/honeywell-sensing-basic-board-mount-pressure-abp-s-1224358.pdf
  */
  double v_sup = 5;

  // convert bin number to a voltage
  double v_out = analogRead(analogPin) * 5.0 / 1024.0;

  // return applied pressure in kPa
  return ((v_out - 0.1 * v_sup) / (0.8 * v_sup)) * P_MAX;

}
