/*
   This code is a modified version of a library that Pololu released for the A4990 Motor Driver.
   This library can be found here: https://github.com/pololu/a4990-motor-shield
*/

/*
   Logic Table from the Driver Board
   Observered - matches table in data sheet.
     IN1/3  IN2/4  OUT1/3  OUT2/4
     (DIR)  (PWM)
     ----------------------------------
      0       0       0       1  CCLOCK (FILL)
      0       1       0       0  BRAKE
      1       0       1       1  BRAKE
      1       1       1       0  CLOCK (VENT)

   PCB DESIGN
   ----------
   Valve 0:
   IN3: Pin 8 (DIR) --> OUT3
   IN4: Pin 9 (PWM) --> OUT4

   Valve 1:
   IN1: Pin 7 (DIR) --> OUT1
   IN2: Pin 6 (PWM) --> OUT2

   Valve 2:
   IN3: Pin 4 (DIR) --> OUT3
   IN4: Pin 5 (PWM) --> OUT4

   Valve 3:
   IN1: Pin 2 (DIR) --> OUT1
   IN2: Pin 3 (PWM) --> OUT2


  D3 and D6 can be changed with TCB register (Section 21 of Atmega4809 datasheet)
  D5 and D9 should also be able to be changed, but it will affect millis() and maybe delay().
*/

#include "A4990ValveInterface.h"
const unsigned char A4990ValveInterface::_V0DIR = 8;
const unsigned char A4990ValveInterface::_V1DIR = 7;
const unsigned char A4990ValveInterface::_V2DIR = 4;
const unsigned char A4990ValveInterface::_V3DIR = 2;

const unsigned char A4990ValveInterface::_V0PWM = 9;
const unsigned char A4990ValveInterface::_V1PWM = 6;
const unsigned char A4990ValveInterface::_V2PWM = 5;
const unsigned char A4990ValveInterface::_V3PWM = 3;

//const unsigned char A4990ValveInterface::_FAULT = 6;/

bool A4990ValveInterface::_flipV0 = false;
bool A4990ValveInterface::_flipV1 = false;
bool A4990ValveInterface::_flipV2 = false;
bool A4990ValveInterface::_flipV3 = false;

A4990ValveInterface::A4990ValveInterface()
{

  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high,
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.

  digitalWrite(_V0PWM, LOW);
  digitalWrite(_V1PWM, LOW);
  digitalWrite(_V2PWM, LOW);
  digitalWrite(_V3PWM, LOW);
  pinMode(_V0PWM, OUTPUT);
  pinMode(_V1PWM, OUTPUT);
  pinMode(_V2PWM, OUTPUT);
  pinMode(_V3PWM, OUTPUT);
  digitalWrite(_V0PWM, LOW);
  digitalWrite(_V1PWM, LOW);
  digitalWrite(_V2PWM, LOW);
  digitalWrite(_V3PWM, LOW);

  digitalWrite(_V0DIR, LOW);
  digitalWrite(_V1DIR, LOW);
  digitalWrite(_V2DIR, LOW);
  digitalWrite(_V3DIR, LOW);
  pinMode(_V0DIR, OUTPUT);
  pinMode(_V1DIR, OUTPUT);
  pinMode(_V2DIR, OUTPUT);
  pinMode(_V3DIR, OUTPUT);
  digitalWrite(_V0DIR, LOW);
  digitalWrite(_V1DIR, LOW);
  digitalWrite(_V2DIR, LOW);
  digitalWrite(_V3DIR, LOW);


}

// speed should be a number between -400 and 400
void A4990ValveInterface::setValve0Speed(int speed)
{
  if (speed <= 0)
  { // vent

    // saturate at -400
    if (speed < -400) {
      speed = -400;
    }

    digitalWrite(_V0DIR, HIGH);
    analogWrite(_V0PWM, -speed * 51 / 80);

  } else if (speed > 0) {
    // fill

    //saturate
    if (speed > 400)
      speed = 400;
    digitalWrite(_V0DIR, LOW);
    analogWrite(_V0PWM,  255 - (speed * 51 / 80));
  }

}

// speed should be a number between -400 and 400
void A4990ValveInterface::setValve1Speed(int speed)
{
  if (speed <= 0)
  { // vent

    // saturate at -400
    if (speed < -400) {
      speed = -400;
    }

    digitalWrite(_V1DIR, HIGH);
    analogWrite(_V1PWM, -speed * 51 / 80);

  } else if (speed > 0) {
    // fill

    //saturate
    if (speed > 400)
      speed = 400;
    digitalWrite(_V1DIR, LOW);
    analogWrite(_V1PWM,  255 - (speed * 51 / 80));
  }

}

// speed should be a number between -400 and 400
void A4990ValveInterface::setValve2Speed(int speed)
{
  if (speed <= 0)
  { // vent

    // saturate at -400
    if (speed < -400) {
      speed = -400;
    }

    digitalWrite(_V2DIR, HIGH);
    analogWrite(_V2PWM, -speed * 51 / 80);

  } else if (speed > 0) {
    // fill

    //saturate
    if (speed > 400)
      speed = 400;
    digitalWrite(_V2DIR, LOW);
    analogWrite(_V2PWM,  255 - (speed * 51 / 80));
  }

}

// speed should be a number between -400 and 400
void A4990ValveInterface::setValve3Speed(int speed)
{
  if (speed <= 0)
  { // vent

    // saturate at -400
    if (speed < -400) {
      speed = -400;
    }

    digitalWrite(_V3DIR, HIGH);
    analogWrite(_V3PWM, -speed * 51 / 80);

  } else if (speed > 0) {
    // fill

    //saturate
    if (speed > 400)
      speed = 400;
    digitalWrite(_V3DIR, LOW);
    analogWrite(_V3PWM,  255 - (speed * 51 / 80));
  }

}

// set speed for all valves
// speed should be a number between -400 and 400
void A4990ValveInterface::setSpeeds(int valve_cmd[4])
{
  setValve0Speed(valve_cmd[0]);
  setValve1Speed(valve_cmd[1]);
  setValve2Speed(valve_cmd[2]);
  setValve3Speed(valve_cmd[3]);
}
