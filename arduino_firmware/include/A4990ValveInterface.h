/*
 * This code is a modified version of a library that Pololu released for the A4990 Motor Driver.
 * This library can be found here: https://github.com/pololu/a4990-motor-shield
 */

#ifndef A4990ValveInterface_h
#define A4990ValveInterface_h

#include "Arduino.h"

class A4990ValveInterface
{
public:
  A4990ValveInterface(); // constructor
  static void setValve0Speed(short speed);
  static void setValve1Speed(short speed);
  static void setValve2Speed(short speed);
  static void setValve3Speed(short speed);
  static void setSpeeds(short valve_cmd[4]);
  //    static boolean get/Fault();

private:
  const static unsigned char _V0DIR;
  const static unsigned char _V1DIR;
  const static unsigned char _V2DIR;
  const static unsigned char _V3DIR;
  const static unsigned char _V0PWM;
  const static unsigned char _V1PWM;
  const static unsigned char _V2PWM;
  const static unsigned char _V3PWM;
  //    const static unsigned char _FAULT;

  static bool _flipV0;
  static bool _flipV1;
  static bool _flipV2;
  static bool _flipV3;
};
#endif
