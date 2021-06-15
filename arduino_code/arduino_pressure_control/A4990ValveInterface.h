/*
 * This code is a modified version of a library that Pololu released for the A4990 Motor Driver.
 * This library can be found here: https://github.com/pololu/a4990-motor-shield
 */

#ifndef A4990ValveInterface_h
#define A4990ValveInterface_h

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__)
#define A4990VALVEINTERFACE_USE_20KHZ_PWM
#endif

#include <Arduino.h>

class A4990ValveInterface
{
  public:
    static void setValve0Speed(int speed);
    static void setValve1Speed(int speed);
    static void setValve2Speed(int speed);
    static void setValve3Speed(int speed);
    static void setSpeeds(int valve_cmd[4]);
    static void flipV0(boolean flip);
    static void flipV1(boolean flip);
    static void flipV2(boolean flip);
    static void flipV3(boolean flip);
//    static boolean get/Fault();

  private:
    static void initPinsAndMaybeTimer();
    const static unsigned char _V0DIR;
    const static unsigned char _V1DIR;
    const static unsigned char _V2DIR;
    const static unsigned char _V3DIR;
    const static unsigned char _V0PWM;
    const static unsigned char _V1PWM;
    const static unsigned char _V2PWM;
    const static unsigned char _V3PWM;
//    const static unsigned c/har _FAULT;

    static boolean _flipV0;
    static boolean _flipV1;
    static boolean _flipV2;
    static boolean _flipV3;

    static inline void init()
    {
      static boolean initialized = false;
      if (!initialized)
      {
        initialized = true;
        initPinsAndMaybeTimer();
      }
    }
};
#endif
