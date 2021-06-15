#include <Wire.h>
#include "A4990ValveInterface.h"

A4990ValveInterface valves;

float p[4] = {0, 0, 0, 0};
float pcmd[4] = {0, 0, 0, 0};
float pbias[4] = {0, 0, 0, 0};

// arrays of 8 bytes used for i2c comms. Each pressure uses 2 bytes (16 bits).
byte pchar[2 * 4];
byte pcmdchar[2 * 4];

//// pins used for driving valves
//int digital_pins [4] = {8, 7, 4, 2};
//int pwm_pins [4] = {9, 6, 5, 3};

int valve_cmd[4] = {0, 0, 0, 0}; //valve commands [-400, 400]
const int vent_cmd[4] = {-400,-400,-400,-400};

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
//  // comment out if not debugging
//  Serial.begin(115200);
  
  int i2c_address;
  i2c_address = geti2caddress();

//  //set up fault pins, input pullup bc no external pull is used.
//  pinMode(EF1_A, INPUT_PULLUP);
//  pinMode(EF2_A, INPUT_PULLUP);
//  pinMode(EF1_B, INPUT_PULLUP);
//  pinMode(EF2_B, INPUT_PULLUP);

  //set up I2C and register event handlers
  Wire.begin(i2c_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // vent all valves at first
  valves.setSpeeds(vent_cmd);
  
  // Wait for 5 seconds for everything to vent out.
  delay(5000);
  prevTime = millis();

  // close all valves now that they have vented.
  valves.setSpeeds(valve_cmd);
}


void loop(void)
{
  myTime = millis();
  dt = (myTime - prevTime) * .001; //calculate time (s) between each loop
  prevTime = myTime; //update previous time

  // digital filter pressure data
  p[0] = filter(p[0], readPressure(A0));
  p[1] = filter(p[1], readPressure(A1));
  p[2] = filter(p[2], readPressure(A2));
  p[3] = filter(p[3], readPressure(A3));

//  //Uncomment to plot filtered data vs unfiltered data
//  Serial.print(myTime);
//  Serial.print("\t");
//  Serial.print(readPressure(A0));
//  Serial.print("\t");
//  Serial.print(p[0]);
//  Serial.print("\t");
//  Serial.print(fir_lp.processReading(readPressure(A0)));
//  Serial.println();

//  // Uncomment to see all pressures plotted together on serial monitor
//  for (int i = 0; i < 4; i++) {
//    Serial.print(p[0]);
//    Serial.print(",");
//  }
//  Serial.println();


  // CONTROL
  for (int i = 0; i < 4; i++)
  {
    // calculate control signals for each pressure
    if (abs(pcmd[i] - p[i]) >= deadband)
    {
      error = pcmd[i] - p[i];

      //      if (errorDot[i] < awl) //Integrator anti-windup scheme
      //      {
      //        integrator[i] = integrator[i] + dt/2*(error+prevError[i]); //Trapezoidal Integration
      //      }
      //      pdot[i] = dirtyDifferentiate(p[i], prevP[i], pdot[i]);
      //      errorDot[i] = dirtyDifferentiate(error, prevError[i], errorDot[i]);
      //
      //      float input_signal = error*kp + integrator[i]*ki - pdot[i]*kd;
      //      float input_signal = error*kp;

      valve_cmd[i] = error * kp;

      //update delayed variables
      prevError[i] = error;
      prevP[i] = p[i];
    }
  }

  // send updated control signals
  valves.setSpeeds(valve_cmd);

  //update p and pcmd in memory with new values of pchar and pcmdchar recieved on i2c
  for (int i = 0; i < 4; i++)
  {
    float_to_two_bytes(p[i], &pchar[i * 2]);
    pcmd[i] = two_bytes_to_float(&pcmdchar[i * 2]);
  }
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
     NOTE: sometimes the sensors read negative pressure
     b/c of their calibration, especially at low pressures.
     So here, we manually set the pressure to 0 if it is negative.
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
