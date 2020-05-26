//#include <SoftwareWire.h>
//SoftwareWire SoftWire(A3,A2);
//#include <Adafruit_ADS1015_softwire.h>
//Adafruit_ADS1015 ads(0x49);

#include <Wire.h>

int i2c_address = 12;

float p[4] = {0, 0, 0, 0};
float pcmd[4] = {0, 0, 0, 0};

char pchar[2 * 4];
char pcmdchar[2 * 4];

int digital_pins [4] = {2,4,7,8};
int pwm_pins [4] = {3,5,6,9};

float valve_cmd[4] = {0,0,0,0};



void setup(void) 
{
  // This speeds up reading from the Arduino's ADC
  // but generally gives worse data (more noisy)
  //bitClear(ADCSRA, ADPS0);
  // These two make little difference in speed
  
  Wire.begin(i2c_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  //SoftWire.setClock(3400000);
  //ads.begin();
  
  for(int i=0; i<4; i++)
  {
    pinMode(digital_pins[i],OUTPUT);
    pinMode(pwm_pins[i], OUTPUT);
    move_valve(digital_pins[i],pwm_pins[i],0);
  }
  
  //Serial.begin(2000000); 
  
  for(int i=0; i<10; i++)
  {
    digitalWrite(13,!digitalRead(13));
    delay(100);
  }
  for(int i=0; i<20; i++)
  {
    digitalWrite(13,!digitalRead(13));
    delay(50);
  }
}


void loop(void) 
{
  //double bias = 165*0;
  //double gain = 100.0/4095.0;
  //p[0] = (ads.readADC_SingleEnded(0)-bias)*gain; // Reading voltage from external ADC
  //p[1] = (ads.readADC_SingleEnded(1)-bias)*gain;
  //p[2] = (ads.readADC_SingleEnded(2)-bias)*gain;
  //p[3] = (ads.readADC_SingleEnded(3)-bias)*gain;

  double bias = 95*0;
  double gain = 100.0/1024.0;
  double a1 = .75;
  double a2 = 1.0-a1;
  p[0] = a1*p[0] + a2*(analogRead(A0)-bias)*gain;
  p[1] = a1*p[1] + a2*(analogRead(A1)-bias)*gain;
  p[2] = a1*p[2] + a2*(analogRead(A2)-bias)*gain;
  p[3] = a1*p[3] + a2*(analogRead(A3)-bias)*gain; 

  float kp = 20.;
  float deadband = .0;
  for(int i=0; i<4; i++)
  { 
    if(abs(pcmd[i]-p[i])>deadband)
    {
      valve_cmd[i] = (pcmd[i]-p[i])*kp;
    }
    move_valve(digital_pins[i],pwm_pins[i],valve_cmd[i]);
  }

  for (int i = 0; i < 4; i++)
  {
    float_to_two_bytes(p[i], &pchar[i * 2]);
    pcmd[i] = two_bytes_to_float(&pcmdchar[i * 2]);
  }
  
  //digitalWrite(13,!digitalRead(13));
}



void move_valve(int digital_pin, int pwm_pin, int speed)
{
  if(speed>255){speed=255;}
  if(speed<-255){speed=-255;}
  
  if(speed>0)
  {
    digitalWrite(digital_pin,LOW);
    analogWrite(pwm_pin,255-speed);
  }
  else
  {
    digitalWrite(digital_pin,HIGH);
    analogWrite(pwm_pin,abs(speed));
  }
}

void receiveEvent(int howMany)
{
  if (howMany > 1)
  {
    Wire.read();
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

float two_bytes_to_float(unsigned char * twobytes)
{
  uint16_t myint;
  memcpy(&myint, twobytes, 2);
  return float(100.0 * myint / 65536.0);
}

void float_to_two_bytes(float myfloat, unsigned char * twobytes)
{
  uint16_t myint = myfloat * 65536.0 / 100.0;
  memcpy(twobytes, &myint, 2);
}
