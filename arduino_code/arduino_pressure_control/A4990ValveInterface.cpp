/*
 * This code is a modified version of a library that Pololu released for the A4990 Motor Driver.
 * This library can be found here: https://github.com/pololu/a4990-motor-shield
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

boolean A4990ValveInterface::_flipV0 = false;
boolean A4990ValveInterface::_flipV1 = false;
boolean A4990ValveInterface::_flipV2 = false;
boolean A4990ValveInterface::_flipV3 = false;

void A4990ValveInterface::initPinsAndMaybeTimer()
{
  // Initialize the pin states used by the motor driver shield
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

  //  pinMode(_FAULT, INPUT_PULLUP);
#ifdef A4990MOTODRIVER_USE_20KHZ_PWM
  // timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
#endif
}

// speed should be a number between -400 and 400
void A4990ValveInterface::setValve0Speed(int speed)
{
  init(); // initialize if necessary

  boolean reverse = 0;

  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400)  // max
    speed = 400;

#ifdef A4990ValveInterface_USE_20KHZ_PWM
  OCR1A = speed;
#else
  analogWrite(_V0PWM, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif

  if (reverse ^ _flipV0) // flip if speed was negative or _flipM1 setting is active, but not both
    digitalWrite(_V0DIR, HIGH);
  else
    digitalWrite(_V0DIR, LOW);
}

// speed should be a number between -400 and 400
void A4990ValveInterface::setValve1Speed(int speed)
{
  init(); // initialize if necessary

  boolean reverse = 0;

  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400)  // max
    speed = 400;

#ifdef A4990ValveInterface_USE_20KHZ_PWM
  OCR1A = speed;
#else
  analogWrite(_V1PWM, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif

  if (reverse ^ _flipV1) // flip if speed was negative or _flipM1 setting is active, but not both
    digitalWrite(_V1DIR, HIGH);
  else
    digitalWrite(_V1DIR, LOW);
}

// speed should be a number between -400 and 400
void A4990ValveInterface::setValve2Speed(int speed)
{
  init(); // initialize if necessary

  boolean reverse = 0;

  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400)  // max
    speed = 400;

#ifdef A4990ValveInterface_USE_20KHZ_PWM
  OCR1A = speed;
#else
  analogWrite(_V2PWM, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif

  if (reverse ^ _flipV2) // flip if speed was negative or _flipM1 setting is active, but not both
    digitalWrite(_V2DIR, HIGH);
  else
    digitalWrite(_V2DIR, LOW);
}

// speed should be a number between -400 and 400
void A4990ValveInterface::setValve3Speed(int speed)
{
  init(); // initialize if necessary

  boolean reverse = 0;

  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400)  // max
    speed = 400;

#ifdef A4990ValveInterface_USE_20KHZ_PWM
  OCR1A = speed;
#else
  analogWrite(_V3PWM, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif

  if (reverse ^ _flipV3) // flip if speed was negative or _flipM1 setting is active, but not both
    digitalWrite(_V3DIR, HIGH);
  else
    digitalWrite(_V3DIR, LOW);
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

void A4990ValveInterface::flipV0(boolean flip)
{
  A4990ValveInterface::_flipV0 = flip;
}

void A4990ValveInterface::flipV1(boolean flip)
{
  A4990ValveInterface::_flipV1 = flip;
}

void A4990ValveInterface::flipV2(boolean flip)
{
  A4990ValveInterface::_flipV2 = flip;
}

void A4990ValveInterface::flipV3(boolean flip)
{
  A4990ValveInterface::_flipV3 = flip;
}

//boolean A4990ValveInterface::getFault()
//{
//  init(); // initialize if necessary
//  return digitalRead(_FAULT) == LOW;
//}
