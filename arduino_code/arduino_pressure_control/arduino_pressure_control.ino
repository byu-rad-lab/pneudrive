//#include <SoftwareWire.h>
//SoftwareWire SoftWire(A3,A2);
//#include <Adafruit_ADS1015_softwire.h>
//Adafruit_ADS1015 ads(0x49);

#include <Wire.h>

float p[4] = {0, 0, 0, 0};
float pcmd[4] = {0, 0, 0, 0};

// arrays of 8 chars used for i2c comms. Each pressure uses 2 chars (bytes).
char pchar[2 * 4];
char pcmdchar[2 * 4];

// pins used for driving valves
int digital_pins [4] = {2,4,7,8};
int pwm_pins [4] = {3,5,6,9};

float valve_cmd[4] = {0,0,0,0};

float myTime = 0;

//// motor driver error pins
//const int EF1_A = 10;
//const int EF2_A = 11;
//const int EF1_B = 20;
//const int EF2_B = 21;

// conversion factor from psi to kpa (ie kpa = psi * psi2kpa)
double const psi2kpa = 6.8947572932;

void setup(void) 
{
    Serial.begin(9600);
  // This speeds up reading from the Arduino's ADC
  // but generally gives worse data (more noisy)
  //bitClear(ADCSRA, ADPS0);
  // These two make little difference in speed

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

  //SoftWire.setClock(3400000);
  //ads.begin();

  //set each digital/pwm pin as output and initialize valves at 0 speed
//  Serial.print("Initializing valves");
//  Serial.println();
  for(int i=0; i<4; i++)
  {
    pinMode(digital_pins[i],OUTPUT);
    pinMode(pwm_pins[i], OUTPUT);
    move_valve(digital_pins[i],pwm_pins[i],0);
  }
}


void loop(void) 
{
  //myTime =micros();
  //double bias = 165*0;
  //double gain = 100.0/4095.0;
  //p[0] = (ads.readADC_SingleEnded(0)-bias)*gain; // Reading voltage from external ADC
  //p[1] = (ads.readADC_SingleEnded(1)-bias)*gain;
  //p[2] = (ads.readADC_SingleEnded(2)-bias)*gain;
  //p[3] = (ads.readADC_SingleEnded(3)-bias)*gain;

  double prev_weight = .5;
  double sensor_weight = 1.0-prev_weight;

  // moving weighted average of pressures
  p[0] = prev_weight*p[0] + sensor_weight*readPressure(A0);
  p[1] = prev_weight*p[1] + sensor_weight*readPressure(A1);
  p[2] = prev_weight*p[2] + sensor_weight*readPressure(A2);
  p[3] = prev_weight*p[3] + sensor_weight*readPressure(A3); 

  // CONTROL
  float kp = 1.;
  float deadband = .0;
  for(int i=0; i<4; i++)
  { 
    if(abs(pcmd[i]-p[i])>=deadband)
    {
      valve_cmd[i] = (pcmd[i]-p[i])*kp; 
    }
    
//    if (i==0){
//        Serial.print(valve_cmd[i],8);
//        Serial.println();
//    }

    move_valve(digital_pins[i],pwm_pins[i],valve_cmd[i]);
  }

  //update p and pcmd in memory with new values of pchar and pcmdchar
  for (int i = 0; i < 1; i++)
  {
    float_to_two_bytes(p[i], &pchar[i * 2]);
    pcmd[i] = two_bytes_to_float(&pcmdchar[i * 2]); 
  }
  
  //Serial.print(micros() - myTime);
  //Serial.println();
}


void move_valve(int digital_pin, int pwm_pin, int speed)
{
  /*
   * speed comes in as a number between -2*pmax*kp and 2*pmax*kp
   * Logic in this function is:
   * if speed is negative -> measured pressure is above command -> vent
   * if speed is positive -> measured pressure is below command -> fill
   */
  
  //saturate
  if(speed>255){speed=255;}
  if(speed<-255){speed=-255;}

  if (digital_pin==2)
  {
    Serial.print(speed);
    Serial.println();
  }

//  Serial.print(speed);
//  Serial.println();
  
  if(speed>0)
  { 
    //fill
    fill(digital_pin, pwm_pin, speed);
  }
  else
  {
    //vent
    vent(digital_pin, pwm_pin, speed);
  }
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
  return float((100.0 * psi2kpa) * myint / 65535.0);
}

void float_to_two_bytes(float myfloat, unsigned char * pchar_twobytes)
{
  // convert float of kPa to bits to send over i2c
  // uint16_t can be 0 to 65535
  uint16_t myint = (myfloat / (100.0 * psi2kpa)) * 65535.0 ;
  
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
  
  one = digitalRead(LED_BUILTIN);
  two = digitalRead(12);

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
   
   double p_max = 100 * psi2kpa; //convert 100 psi sensor max to kPa
   double v_sup = 5;
   
   // convert bin number to a voltage
   double v_out = analogRead(analogPin) * 5.0/1024.0;

   //just for prototyping with pot, comment out with real sensor
   if (v_out>4.5){v_out=4.5;}
   if (v_out<0.5){v_out=0.5;}

   if (analogPin == A0){
    //Serial.print(v_out);
    //Serial.println();
   }

   // return applied pressure in kPa
   return ((v_out - 0.1*v_sup)/(0.8*v_sup)) * p_max;
 
}

void fill(int digital_pin, int pwm_pin, int speed)
{
  /*
   * speed input is (0,255]
   */
  digitalWrite(digital_pin,LOW);
  analogWrite(pwm_pin,255-speed);
}

void vent(int digital_pin, int pwm_pin, int speed)
{
  /*
   * speed input is [-255,0]
   */
  digitalWrite(digital_pin,HIGH);
  analogWrite(pwm_pin,abs(speed));
}
