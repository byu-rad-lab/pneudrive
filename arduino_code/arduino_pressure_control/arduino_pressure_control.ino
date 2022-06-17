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

A4990ValveInterface valves;

float p[4] = {0, 0, 0, 0};
float pcmd[4] = {0, 0, 0, 0};

// arrays of 8 bytes used for i2c comms. Each pressure uses 2 bytes (16 bits).
// NOTE: all pressures sent over i2c are in kPa (gauge).
const int BYTES_PER_PRESSURE = 2;
byte pchar[BYTES_PER_PRESSURE * 4];
byte pcmdchar[BYTES_PER_PRESSURE * 4];

int valve_cmd[4] = {0, 0, 0, 0};
int VENT_CMD[4] = { -400, -400, -400, -400};
int FILL_CMD[4] = {400, 400, 400, 400};

// conversion factor from psi to kpa (ie kpa = psi * PSI2KPA)
double const PSI2KPA = 6.8947572932;
double const P_MAX = 100 * PSI2KPA;

//'Global' variables that are important for control
// only proportional control b/c these dynamics are essentially a stable first order system and I don't
// want to add a destabilizing integrator on this low level control
float myTime = 0.0;
float prevTime = 0.0; // Last time the loop was entered
float kp = 0.7 / (10 * PSI2KPA); //this kp means desired current will rail at .7 amps at a pressure error of 10 psi.
// denominator is set by the pressure error which will cause the input to saturate.
float deadband = 0.0; // kpa, controller will not act on error less than deadband
float error = 0.0;
float dt = 0;

// motor driver error pins
const int EF1_A = 10;
const int EF2_A = 11;
const int EF1_B = 20;
const int EF2_B = 21;

// timer stuff
int const PRESCALER = 32;
int const ONE_SECOND = 32000;

void setup(void)
{

  speedupPWM();
  // comment out if not debugging
//  Serial.begin(1000000);

  int i2c_address;
  i2c_address = geti2caddress();

  // set up fault pins, input pullup bc no external pull is used.
  pinMode(EF1_A, INPUT_PULLUP);
  pinMode(EF2_A, INPUT_PULLUP);
  pinMode(EF1_B, INPUT_PULLUP);
  pinMode(EF2_B, INPUT_PULLUP);

  // set up I2C and register event handlers
  Wire.begin(i2c_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  //  // uncomment to test valves
  //  valves.setValve0Speed(-200);
  //  valves.setValve1Speed(-200);
  //  valves.setValve2Speed(-200);
  //  valves.setValve3Speed(-200);

  // vent all valves at first
  valves.setSpeeds(VENT_CMD);

  // Wait for 5 seconds for everything to vent out.
  custom_delay(5);
  prevTime = micros();

  // close all valves now that they have vented.
  valves.setSpeeds(valve_cmd);
}

void loop(void)
{

  myTime = micros();
  dt = ((myTime - prevTime) / PRESCALER) * .000001; // calculate time (s) between each loop
  prevTime = myTime;                             // update previous time


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

  // PROPORTIONAL CONTROL on each valve
  for (int i = 0; i < 4; i++)
  {
    // calculate control signals for each pressure
    if (abs(pcmd[i] - p[i]) >= deadband)
    {
      error = pcmd[i] - p[i];

      valve_cmd[i] = current2PWM(kp * error);

      //      if (i == 0) {
      //        Serial.print(valve_cmd[i]);
      //        Serial.println();
      //      }
    }
  }

  // send updated control signals [-400,400]
  valves.setSpeeds(valve_cmd);

  // update p and pcmd in memory with new values of pchar and pcmdchar recieved on i2c
  for (int i = 0; i < 4; i++)
  {
    float_to_two_bytes(p[i], &pchar[i * BYTES_PER_PRESSURE]);
    pcmd[i] = two_bytes_to_float(&pcmdchar[i * BYTES_PER_PRESSURE]);
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

  return b * prev + a * input;
}

void receiveEvent(int howMany)
{
  if (howMany > 1)
  {
    // extract correct number of bytes of pressure command (2 bytes/chamber)
    for (int j = 0; j < 4 * BYTES_PER_PRESSURE; j++)
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

float two_bytes_to_float(byte * pcmdchar)
{
  if (BYTES_PER_PRESSURE == 1)
  {
    uint8_t myint;
    memcpy(&myint, pcmdchar, BYTES_PER_PRESSURE);
    return float((100.0 * PSI2KPA) * myint / 255.0);
  }
  else if (BYTES_PER_PRESSURE == 2)
  {
    uint16_t myint;
    memcpy(&myint, pcmdchar, BYTES_PER_PRESSURE);
    return float((100.0 * PSI2KPA) * myint / 65535.0);
  }
}

void float_to_two_bytes(float myfloat, byte * pchar)
{
  /*
     NOTE: sometimes the sensors read negative pressure
     b/c of noise, especially at low pressures.
     So here, we manually set the pressure to 0 if it is negative.
  */
  // convert float of kPa to bits to send over i2c
  // uint16_t can be 0 to 65535

  if (myfloat < 0)
  {
    myfloat = 0;
  }

  // memcpy(pointer to destination, pointer to source, # bytes to copy)
  // copy BYTES_PER_PRESSURE bytes of data located @ myint to location of pchar

  if (BYTES_PER_PRESSURE == 1)
  {
    uint8_t myint = (myfloat / (100.0 * PSI2KPA)) * 255.0;
    memcpy(pchar, &myint, BYTES_PER_PRESSURE);
  }
  else if (BYTES_PER_PRESSURE == 2)
  {
    uint16_t myint = (myfloat / (100.0 * PSI2KPA)) * 65535.0;
    memcpy(pchar, &myint, BYTES_PER_PRESSURE);
  }
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

  if (one == LOW && two == LOW)
  {
    i2caddr = 0xA;
  }
  else if (one == HIGH && two == LOW)
  {
    i2caddr = 0xB;
  }
  else if (one == HIGH && two == HIGH)
  {
    i2caddr = 0xC;
  }
  else if (one == LOW && two == HIGH)
  {
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

void custom_delay(int seconds)
{
  for (int s = 0; s < seconds; s++)
  {
    delay(ONE_SECOND);
  }
}
