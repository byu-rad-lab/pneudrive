//#include <SoftwareWire.h>
//SoftwareWire SoftWire(A3,A2);
//#include <Adafruit_ADS1015_softwire.h>
//Adafruit_ADS1015 ads(0x49);

#include <Wire.h>

int i2c_address = 11;

float p[4] = {0, 0, 0, 0};
float pcmd[4] = {0, 0, 0, 0};
char pchar[2 * 4];
char pcmdchar[2 * 4];

int digital_pins [4] = {2,4,7,8};
int pwm_pins [4] = {3,5,6,9};



void setup(void) 
{
  // This speeds up reading from the Arduino's ADC
  bitClear(ADCSRA, ADPS0);
  // These two make little difference in speed
  //bitSet(ADCSRA, ADPS1);
  //bitClear(ADCSRA, ADPS2); // This one causes bad readings
  
  Wire.begin(i2c_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  //SoftWire.setClock(3400000);
  //ads.begin();
  
  for(int i=0; i<4; i++)
  {
    pinMode(digital_pins[i],OUTPUT);
    pinMode(pwm_pins[i], OUTPUT);
    move_valve(digital_pins[i],pwm_pins[i],0,0);
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
  //double bias = 165;
  //double gain = 80.0/1115.0;
  //p[0] = (ads.readADC_SingleEnded(0)-bias)*gain; // Reading voltage from external ADC
  //p[1] = (ads.readADC_SingleEnded(1)-bias)*gain;
  //p[2] = (ads.readADC_SingleEnded(2)-bias)*gain;
  //p[3] = (ads.readADC_SingleEnded(3)-bias)*gain;

  double bias = 95*0;
  //double gain = 50.0/400.0;
  double gain = 100.0/1024.0;
  p[0] = .5*p[0] + .5*(analogRead(A0)-bias)*gain;
  p[1] = .5*p[1] + .5*(analogRead(A1)-bias)*gain;
  p[2] = .5*p[2] + .5*(analogRead(A2)-bias)*gain;
  p[3] = .5*p[3] + .5*(analogRead(A3)-bias)*gain;

  //move_valve(digital_pins[0],pwm_pins[0],100,1);
  

  for(int i=0; i<4; i++)
  {
    bool dir;
    if(pcmd[i]-p[i]>0){dir=1;}
    else{dir=0;}
    int cmd = abs(pcmd[i]-p[i])*200.0;
    if(cmd<50){cmd=0;}
    move_valve(digital_pins[i],pwm_pins[i],cmd,dir);
  }

  for (int i = 0; i < 4; i++)
  {
    float_to_two_bytes(p[i], &pchar[i * 2]);
    pcmd[i] = two_bytes_to_float(&pcmdchar[i * 2]);
  }

//  for(int i=0; i<4; i++)
//  {
//    Serial.print(pcmd[i]);
//    Serial.print("    ");
//  }
//  Serial.println();

  digitalWrite(13,!digitalRead(13));
}














void move_valve(int digital_pin, int pwm_pin, int speed, bool forward)
{
  if(speed>255){speed=255;}
  if(speed<0){speed=0;}
  
  if(forward)
  {
    digitalWrite(digital_pin,LOW);
    analogWrite(pwm_pin,255-speed);
  }
  else
  {
    digitalWrite(digital_pin,HIGH);
    analogWrite(pwm_pin,speed);
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
    //Serial.println("Receive Event");
  }
  else
  {
    Wire.read();
  }
}

void requestEvent()
{
  //Serial.println("Request Event"); 
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
