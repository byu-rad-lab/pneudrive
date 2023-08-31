#include <Wire.h>
#include "A4990ValveInterface.h"

/*
  NOTE ON VALVE COMMAND SIGNALS
  -----------------------------
  Valve PWM commands are on the interval [-400,400].

  -400 corresponds to commanding -1 amp (vent) and 400 corresponds to +1 amp (fill).
  The driver boards actively limit current to 0.7 amps however,
  so any command above 0.7 amps will result in 0.7 amps on the hardware. See current2PWM() function below.

  Negative numbers mean vent and positive mean fill.
  If the valve is flipped, the wires need to be switched.

*/

/*
  IMPORTANT NOTE ON TIMER FREQUENCIES
  ------------------------------------
  On the Arduino Nano Every (ATmega 4809 chipset), there are two timers related to the PWM pins we
  using: TCA for D5 and D9 and TCB for D3 and D6.

  The default pwm freq for these pins is 975 Hz = fclk_per/64 (where fclk_per is the peripheral clock,
  and is set to 62.4kHz by default. This freq causes a really annoying hum when the valves are being
  controlled. To get rid of this, we prescale the timers to 31.2kHz = fclk_per/2 in speedupPWM() because
  31.2 kHz is well outside the range of what a human can hear.

  TCB can be changed like this without causing any issues. TCA, on the other hand, is used by the
  built-in delay() and millis() functions. Since we changed the freq by a factor of 32 (PRESCALER),
  this means that millis() now reports 32 times the actual time passed (e.g. millis()/PRESCALER is the
  actual number of milliseconds that have passed).

  Another consequence of changing TCA is that delay() now (for some reason) can only delay up to
  32,768 (2^16/2) which corresponds to an actual delay of 32768/32 = 1024 ms. So, there is a custom
  delay function defined custom_delay() which allows you to delay for longer than 1024 ms. Since
  delay() function isn't used in the control loop, this is adequate for our needs.
*/

// ANYWHERE WE SEE FLOAT, CHANGE IT


A4990ValveInterface valves;

int rs485_address = getrs485address();

unsigned char pressure_command_bytes[8]; //comes from odroid
unsigned short pressure_command_shorts[4];
unsigned short pressure_state_shorts[4] = {666, 777, 888, 999};
unsigned char pressure_state_bytes[9] = {rs485_address, 0x02, 0x9A, 0x03, 0x09, 0x03, 0x78, 0x03, 0xE7}; // sends to odroid

unsigned short p[4] = {0, 0, 0, 0};
unsigned short pcmd[4] = {0, 0, 0, 0};

// Keep, make it explicitly a short
short valve_cmd[4] = {0, 0, 0, 0};
short VENT_CMD[4] = { -400, -400, -400, -400};
short FILL_CMD[4] = {400, 400, 400, 400};

// Probably move to odroid. We just want integer math
// conversion factor from psi to kpa (ie kpa = psi * PSI2KPA)
double const PSI2KPA = 6.8947572932;
double const P_MAX = 100 * PSI2KPA;

// IGNORE
//'Global' variables that are important for control
// only proportional control b/c these dynamics are essentially a stable first order system and I don't
// want to add a destabilizing integrator on this low level control
float myTime = 0.0;
float prevTime = 0.0; // Last time the loop was entered

/*  int bus;
 * Sensors in this configuration have noise magnitude of about 0.5 psi. 
 * Making railPSI < 2 made the arm very jittery. 
 * railPSI=2 seems to track well.
 * railPSI>5 was pretty sluggish tracking and seemed to exacerbate valve nonlinearities with small commands.
 *  
 */
float railPSI = 2;
float kp = 0.7 / (railPSI * PSI2KPA); //this kp means desired current will rail at .7 amps at a pressure error of >= railPSI.


// denominator is set by the pressure error which will cause the input to saturate.
// KEEP DEADBAND
short deadband = 0; // kpa, controller will not act on error less than deadband 
short error = 0;

// timer stuff (KEEP)
int const PRESCALER = 32;
int const ONE_SECOND = 32000;

//===============================================================================
//  Initialization
//===============================================================================
void setup()
{

  speedupPWM();
  // comment out if not debugging
  //  Serial.begin(1000000);

  Serial1.begin(1000000);  // RS485 Serial port
  pinMode(LED_BUILTIN, OUTPUT);

  //  // uncomment to test valves
  //  valves.setValve0Speed(-200);
  //  valves.setValve1Speed(-200);
  //  valves.setValve2Speed(-200);
  //  valves.setValve3Speed(-200);

  // vent all valves at first
  valves.setSpeeds(VENT_CMD);

  // Wait for 5 seconds for everything to vent out.
  custom_delay(5);

  // close all valves now that they have vented.
  valves.setSpeeds(valve_cmd);
}

//===============================================================================
//  Main
//===============================================================================
void loop()
{
  if (Serial1.available() == 9) // If data has come in from Odroid
  {
    if (Serial1.read() == rs485_address) 
    {
      // if address is correct, read in the rest of the message (8 bytes)
      for (int i=0; i<8; i++) {
        pressure_command_bytes[i] = Serial1.read();
      }

      //convert the bytes to shorts to be used by the arduino
      byteToShorts(pressure_command_shorts, pressure_command_bytes);

      // do something with the pressure commands here
      for (int i = 0; i < pressure_command_shorts.size(); i++)
      {
        // calculate control signals for each pressure
        if (abs(pressure_command_shorts[i] - pressure_state_shorts[i]) >= deadband)
        {
          error = pressure_command_shorts[i] - pressure_state_shorts[i];

          valve_cmd[i] = current2PWM(kp * error); 
        }
      }

      // send updated control signals [-400,400]
      valves.setSpeeds(valve_cmd);

      // update current pressure_state
      pressure_state_shorts[0] = filter(pressure_state_shorts[0], A0);
      pressure_state_shorts[1] = filter(pressure_state_shorts[1], A1);
      pressure_state_shorts[2] = filter(pressure_state_shorts[2], A2);
      pressure_state_shorts[3] = filter(pressure_state_shorts[3], A3);

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
      
      // convert the current state into bytes
      shortToBytes(pressure_state_shorts, pressure_state_bytes);

      // still not sure why, but this is the smallest delay that allows the odroid to receive data
      // delay from rs485 boards is 120 us, and packet takes 90 us (120-90 = 30 us), but sending to RX buffer takes about 30 us. so add just a bit.
      delayMicroseconds(5);

      // Send current state back to odroid
      for (int i=0; i<9; i++) {
        Serial1.write(pressure_state_bytes[i]);
      }
    }
    else
    {
      while (Serial1.available())
      {
        Serial1.read();
      }
    }
  }
}


void byteToShorts(unsigned short *short_array, unsigned char *byte_array) {
  //Function to convert array of 8 bytes to array of 4 shorts
  unsigned int byteLength = 8;
  for (unsigned int i=0; i<byteLength; i+=2) {
    short_array[i/2] = ((short)byte_array[i] << 8) | byte_array[i+1];
  }
}

void shortToBytes(unsigned short *short_array, unsigned char *byte_array) {
  // Function to convert array of 4 shorts to array of 8 bytes (doesn't change first byte because of address byte)
  int shortLength = 4;
  for (int i=0; i<shortLength; i++) {
    int byteIndex = i*2+1;
    unsigned char *bytePtr = (unsigned char *)&short_array[i];
    byte_array[byteIndex] = bytePtr[1]; // Most significant byte
    byte_array[byteIndex+1] = bytePtr[0]; // Least significant byte
  }
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

  return b * prev + a * input(v_out - 0.1 * v_sup) / (0.8 * v_sup)) * P_MAX;
}


int getrs485address()
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

  int rs485addr;
  int one;
  int two;

  // b/c we are using pin 13 as an input, the LED_BUILTIN cannot be used.
  pinMode(LED_BUILTIN, INPUT);
  pinMode(12, INPUT);

  one = digitalRead(12);
  two = digitalRead(LED_BUILTIN);

  if (one == LOW && two == LOW)
  {
    rs485addr = 0xA;
  }
  else if (one == HIGH && two == LOW)
  {
    rs485addr = 0xB;
  }
  else if (one == HIGH && two == HIGH)
  {
    rs485addr = 0xC;
  }
  else if (one == LOW && two == HIGH)
  {
    rs485addr = 0xD;
  }
  return rs485addr;
}

int current2PWM(double current)
{
  /*
    From enfield valve measurements: current = .004*PWM

    Note, this model is only valid in the linear regime which is PWM=[-175,175].
    Outside of this, the current is maxed out by the driver boards at ~0.7 amps.
    That is, any PWM command outside of [-175,175] will be saturated at .7 amps.
  */
  int PWM = int(current / .004);
  return PWM;
}
// LEAVE
void speedupPWM()
{
  cli(); // Disable Interrupts

  /* see section 20.5.1 of http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-4809-Data-Sheet-DS40002173A.pdf
     for this register settings. Note that by default, fclk_per = 62.4 kHz.
     Humans can hear 20 Hz to 20 kHz, so a 31.2kHz freq should be out of hearing range.
     Also see section 5.4.1 for syntax used here.
     | is "bitwise or" commonly used to set multiple bits at once in a register.
     TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm --> 0,0,1,0 OR 0,0,0,1 = 0,0,1,1

     READ TIMER FREQUENCY NOTE AT TOP
  */

  // TCB for D3 and D6
  TCB0.CTRLA = (TCB_CLKSEL_CLKDIV2_gc) | (TCB_ENABLE_bm);
  // TCA for D5 and D9
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;

  sei(); // Enable Interrupts
}
// LEAVE
void custom_delay(int seconds)
{
  for (int s = 0; s < seconds; s++)
  {
    delay(ONE_SECOND);
  }
}
