//#include <SoftwareWire.h>
//SoftwareWire SoftWire(A3,A2);
//#include <Adafruit_ADS1015_softwire.h>
//Adafruit_ADS1015 ads(0x49);

#include <Wire.h>

float p[4] = {0, 0, 0, 0};
float pcmd[4] = {0, 0, 0, 0};
float pbias[4] = {0, 0, 0, 0};

// arrays of 8 chars used for i2c comms. Each pressure uses 2 chars (bytes).
char pchar[2 * 4];
char pcmdchar[2 * 4];

// pins used for driving valves
int digital_pins [4] = {8,7,4,2};
int pwm_pins [4] = {9,6,5,3};

int valve_cmd[4] = {0,0,0,0}; //valve commands [0, 255]

//'Global' variables that are important for control
float myTime = 0.0;
//float startTime = 0.0;
float prevTime = 0.0; //Last time the loop was entered
float integrator[4] = {0, 0, 0, 0};
float derivative[4] = {0, 0, 0, 0};
float prevError[4] = {0, 0, 0, 0}; //error at the previous timestep
float errorDot[4] = {10000.0, 10000.0, 10000.0, 10000.0}; //Derivative of the error, initialized high so that the integrator
                          //will start off the bat.
float awl = 25.0; //Anti-Windup limit.  If the error is changing less than this value, the 
                  //integrator won't continue to increase. 

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
  Serial.begin(9600);
  // This speeds up reading from the Arduino's ADC
  // but generally gives worse data (more noisy)
  //bitClear(ADCSRA, ADPS0);
  // These two make little difference in speed

  int i2c_address;
  i2c_address = geti2caddress();

//  Serial.print(i2c_address, HEX);

  //set up fault pins, input pullup bc no external pull is used.
  pinMode(EF1_A, INPUT_PULLUP);
  pinMode(EF2_A, INPUT_PULLUP);
  pinMode(EF1_B, INPUT_PULLUP);
  pinMode(EF2_B, INPUT_PULLUP);

  //set up I2C and register event handlers
  Wire.begin(i2c_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  //SoftWire.setClock(3400000);
  //ads.begin();

  //set each digital/pwm pin as output and initialize valves at 0 speed
//  Serial.print("Initializing valves");
//  Serial.println();
  for(int i=0; i<4; i++)
  {
    pinMode(digital_pins[i],OUTPUT);
    pinMode(pwm_pins[i], OUTPUT);
    vent(digital_pins[i],pwm_pins[i],0);
  }
  prevTime = micros();
}


void loop(void) 
{
  myTime = micros(); //TODO: This overflows after an hour, should we switch to millis?
                     //Also for strictest accuracy we may want to make this an array
                     //and put it in the for loop down below.
  //double bias = 165*0;
  //double gain = 100.0/4095.0;
  //p[0] = (ads.readADC_SingleEnded(0)-bias)*gain; // Reading voltage from external ADC
  //p[1] = (ads.readADC_SingleEnded(1)-bias)*gain;
  //p[2] = (ads.readADC_SingleEnded(2)-bias)*gain;
  //p[3] = (ads.readADC_SingleEnded(3)-bias)*gain;

  // digital filter pressure data
  p[0] = filter(p[0], readPressure(A0));
  p[1] = filter(p[1], readPressure(A1));
  p[2] = filter(p[2], readPressure(A2));
  p[3] = filter(p[3], readPressure(A3));

//  //Uncomment to plot filtered data vs unfiltered data
//  Serial.print(readPressure(A0));
//  Serial.print("\t");
//  Serial.print(p[0]);
//  Serial.println();

  // Comment in to see all pressures plotted together on serial monitor
//  for (int i = 0; i<4; i++){
//    Serial.print(p[i]/PSI2KPA);
//    Serial.print("\t");
//  }
//  Serial.println();


  // CONTROL
  float kp = 1.0;  //TODO: Should each valve have a set of gains?
  float ki = 0.001;
  float kd = 0.5;
  float sigma = .002; // Dirty Derivative Gain, should take derivatives on signals with 
                     // content less than 1/sigma rad/s. For more info see byu controlbook pg 147
  float deadband = 5.0; // kpa
  float error = 0.0;
  float Ts = myTime-prevTime; //calculate time between errors
  prevTime = myTime; //update previous time. TODO:May want to make this an array and add
                     //to the for loop. See myTime.
  for(int i=0; i<4; i++)
  { 
    if(abs(pcmd[i]-p[i])>=deadband)
    {
      float error = pcmd[i] - p[i];
      
      //integrator += error; Left Hand integration
      if (errorDot[i]>awl) //Integrator anti-windup scheme
      {
        integrator[i] = integrator[i] + Ts/2*(error+prevError[i]); //Trapezoidal Integration
      }
      derivative[i] = ((2*sigma-Ts)/(2*sigma+Ts))*derivative[i]+(2/(2*sigma+Ts))*(error-prevError[i]);
      prevError[i] = error;
//      if (i==0){Serial.print(integrator);}
      
      float input_signal = error*kp + integrator[i]*ki + derivative[i]*kd;

      //map input signal to duty cycle percentage to fill (+) or vent (-)
      valve_cmd[i] = map(input_signal, -P_MAX, P_MAX, -100, 100); 
//      Serial.print(valve_cmd[i]);
    }

//    Serial.print(p[i]);
    //Serial.print("\t");
    move_valve(digital_pins[i],pwm_pins[i],valve_cmd[i]);
  }
//  Serial.print(integrator);
//  Serial.println();

  //update p and pcmd in memory with new values of pchar and pcmdchar
  for (int i = 0; i < 4; i++)
  {
    float_to_two_bytes(p[i], &pchar[i * 2]);
    pcmd[i] = two_bytes_to_float(&pcmdchar[i * 2]); 
  }
  
  Serial.print(micros() - myTime);
  Serial.println();
}


void move_valve(int digital_pin, int pwm_pin, int duty_cycle)
{
  /*
   * speed comes in as a number between -100% and +100%
   * Logic in this function is:
   * if speed is positive -> measured pressure is below command -> fill
   * if speed is negative -> measured pressure is above command -> vent
   * 
   * 0% duty cycle means valve is closed.
   */

//  if (digital_pin==8)
//  {
//    Serial.print(speed);
//    Serial.println();
//  }
  
  if(duty_cycle>0)
  { 
    //fill
    fill(digital_pin, pwm_pin, duty_cycle);
  }
  else
  {
    //vent
    vent(digital_pin, pwm_pin, duty_cycle);
  }
}

void fill(int digital_pin, int pwm_pin, int duty_cycle)
{
  /*
   * speed is (0, 100], map range to (0, 255].
   * NOTE: this cooresponds to direction that the valves are plugged in.
   * Looking at the top of the connector (where the metal is),
   * the brown wire should be on the left, and the blue on the right.
   */


  int speed = map(duty_cycle, 0, 100, 0, 255);
  digitalWrite(digital_pin,HIGH);
  analogWrite(pwm_pin,speed);
}

void vent(int digital_pin, int pwm_pin, int duty_cycle)
{
   /*
   * speed is [-100, 0], map range to (0, 255].
   */
  int speed = map(abs(duty_cycle), 0, 100, 0, 255); 
  digitalWrite(digital_pin,LOW);
  analogWrite(pwm_pin,speed);
}

float filter(float prev, float input)
{
  /* 
   *  This function implements a first order low pass filter
   *  with a cutoff frequency of 50 Hz. First order hold discrete implementation with dt=.001.
   *  First order filter is of form:
   *  a / (z - b)
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

float two_bytes_to_float(unsigned char * pcmdchar_twobytes)
{
  uint16_t myint;
  memcpy(&myint, pcmdchar_twobytes, 2);
  return float((100.0 * PSI2KPA) * myint / 65535.0);
}

void float_to_two_bytes(float myfloat, unsigned char * pchar_twobytes)
{

  /*
   * We have an issue here, sometimes the sensors
   * read negative pressure b/c of the error band. 
   * We can calibrate, set limits in read pressure function, etc.
   * Not sure what best thing to do is. 
   */
  // convert float of kPa to bits to send over i2c
  // uint16_t can be 0 to 65535

  if (myfloat < 0){
    myfloat=0;
  }
  
  uint16_t myint = (myfloat / (100.0 * PSI2KPA)) * 65535.0;
  
  // memcpy(pointer to destination, pointer to source, # bytes to copy)
  // copy 2 bytes of data located @ myint to location of pchar_twobytes
  memcpy(pchar_twobytes, &myint, 2);
}

int geti2caddress()
{
  /* Addresses from dip switch
   *  a number indicates a HIGH value, _ is LOW
   * PIN | Addr
   * ---------
   * _ _ = 0xA
   * 1 _ = 0xB
   * 1 2 = 0xC
   * _ 2 = 0xD
  */

  int i2caddr;
  int one;
  int two;

  // b/c we are using pin 13 as an input, the LED_BUILTIN cannot be used.
  pinMode(LED_BUILTIN, INPUT);
  pinMode(12, INPUT);

  one = digitalRead(12);
  two = digitalRead(LED_BUILTIN);

  if (one == LOW && two == LOW){
    i2caddr = 0xA;
  } else if (one == HIGH && two == LOW){
    i2caddr = 0xB;
  } else if (one == HIGH && two == HIGH){
    i2caddr = 0xC;
  } else if (one == LOW && two == HIGH){
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
   * ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
   * Analog reference is 5V, so 0=0v and 1023=5v
   * Conversion is taken from Fig. 3 (transfer function A) in pressure sensor datasheet
   * https://www.mouser.com/datasheet/2/187/honeywell-sensing-basic-board-mount-pressure-abp-s-1224358.pdf
   */
   double v_sup = 5;
   
   // convert bin number to a voltage
   double v_out = analogRead(analogPin) * 5.0/1024.0;

//   //just for prototyping with pot, comment out with real sensor
//   if (v_out>4.5){v_out=4.5;}
//   if (v_out<0.5){v_out=0.5;}

//   if (analogPin == A1){
//    Serial.print(v_out);
//    Serial.println();
//   }

   // return applied pressure in kPa
   return ((v_out - 0.1*v_sup)/(0.8*v_sup)) * P_MAX;
 
}
