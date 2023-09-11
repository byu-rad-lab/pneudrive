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

// Keep, make it explicitly a short
short valve_cmd[4] = {0, 0, 0, 0};
short VENT_CMD[4] = {-400, -400, -400, -400};
short FILL_CMD[4] = {400, 400, 400, 400};

/*  int bus;
   Sensors in this configuration have noise magnitude of about 0.5 psi.
   Making railPSI < 2 made the arm very jittery.
   railPSI=2 seems to track well.
   railPSI>5 was pretty sluggish tracking and seemed to exacerbate valve nonlinearities with small commands.

*/
// denominator is set by the pressure error which will cause the input to saturate.
const unsigned short MAX_INPUT = 400;
unsigned short saturation_error = 50; // TUNE ME, lower is more aggressive :)
unsigned short kp = MAX_INPUT / saturation_error;

// timer stuff (KEEP)
#define PRESCALER 32
#define ONE_SECOND 32000

#define BYTES_IN_PACKET 10

byte outgoingBytes[BYTES_IN_PACKET] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte incomingDataBytes[BYTES_IN_PACKET - 2] = {0, 0, 0, 0, 0, 0, 0, 0};

unsigned short outgoingShorts[BYTES_IN_PACKET / 2] = {0, 0, 0, 0, 0};

unsigned short rs485_address = 0x0000;
unsigned short pressure_commands[4] = {0, 0, 0, 0};
unsigned short pressure_data[4] = {0, 0, 0, 0};

void byteToShorts(unsigned short *short_array, const byte *byte_array)
{
  // Function to convert array of 8 bytes to array of 4 shorts
  unsigned int byteLength = 8;
  for (size_t i = 0; i < byteLength; i += 2)
  {
    int byteIndex = i;
    short_array[i / 2] = ((short)byte_array[byteIndex] << 8) | byte_array[byteIndex + 1];
  }
}

void shortToBytes(const unsigned short *short_array, byte *byte_array)
{
  // Function to convert array of 5 shorts to array of 10 bytes, but in big endian
  int shortLength = 5;
  for (size_t i = 0; i < shortLength; i++)
  {
    int byteIndex = i * 2;
    unsigned char *bytePtr = (unsigned char *)&short_array[i];

    // convert from little endian to big endian for odroid
    byte_array[byteIndex] = bytePtr[1];     // LSB
    byte_array[byteIndex + 1] = bytePtr[0]; // MSB
  }
}

void handleIncomingBytes()
{
  Serial.print("start bytes: ");
  Serial.println(Serial1.available(), DEC);
  byte firstByte = 0;

  if (Serial1.available() == 9)
  {
    Serial.println("Dropped Byte!");
  }

  // Serial.println(Serial1.available(), DEC);
  while (Serial1.available() > 0)
  {
    // read first 2 bytes of transmission to see if this device is being addressed
    // if address is never found, this will simply empty the RX buffer.

    byte secondByte = Serial1.read();

    unsigned short short1 = (firstByte << 8) | secondByte; // arduino is little endian

    Serial.println(firstByte, HEX);
    Serial.println(secondByte, HEX);
    Serial.println();

    if (short1 == rs485_address)
    {
      Serial.println("address found");
      // if this device address is found, save the next 8 bytes because they contain pressure commands sent from controller
      if (Serial1.readBytes(incomingDataBytes, BYTES_IN_PACKET - 2) == 8)
      {
        // print each bytes in incoming data bytes
        for (int i = 0; i < 8; i++)
        {
          Serial.println(incomingDataBytes[i], HEX);
        }
        Serial.println("read 8 bytes");
        // respond with pressure data
        Serial1.write(outgoingBytes, BYTES_IN_PACKET);
        // if the incoming array was meant for this device, save it for use in control
        byteToShorts(pressure_commands, incomingDataBytes);

        firstByte = 0;
      }
    }
    else
    {
      firstByte = secondByte;
    }
  }
  // Serial.println(Serial1.available(), DEC);
  Serial.println("done");
}

void readPressureData()
{
  // input voltage range from pressure sensors is .1Vsup - .9Vsup, which is .5V to 4.5V
  // These voltages correspond to about 102 and 921 on the arduino analog input.
  pressure_data[0] = analogRead(A0);
  pressure_data[1] = analogRead(A1);
  pressure_data[2] = analogRead(A2);
  pressure_data[3] = analogRead(A3);

  for (int i = 0; i < 4; i++)
  {
    outgoingShorts[i + 1] = pressure_data[i];
  }
}

//
// float filter(float prev, float input)
//{
//  /*
//      This function implements a first order low pass filter
//      with a cutoff frequency of 50 Hz. First order hold discrete implementation with dt=.001.
//      First order filter is of form:
//      a / (z - b)
//  */
//
//  float a = .2696;
//  float b = .7304;
//
//  return b * prev + a * input(v_out - 0.1 * v_sup) / (0.8 * v_sup)) * P_MAX;
//}
void custom_delay(int seconds)
{
  for (int s = 0; s < seconds; s++)
  {
    delay(ONE_SECOND);
  }
}

int getrs485address()
{
  /* Addresses from dip switch
      a number indicates a DIP switch flipped ON
     PIN |Addr
     ---------
     _ _ = 0
     1 _ = 1
     _ 1 = 2
     1 1 = 3

     Note that when the DIP switch is on, the actual voltage is LOW. When it is OFF,
     the voltage is pulled HIGH.
  */

  unsigned short rs485addr;
  int one;
  int two;

  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);

  one = digitalRead(10);
  two = digitalRead(11);

  if (one == HIGH && two == HIGH)
  {
    rs485addr = 0xFFFF - 0;
  }
  else if (one == LOW && two == HIGH)
  {
    rs485addr = 0xFFFF - 1;
  }
  else if (one == HIGH && two == LOW)
  {
    rs485addr = 0xFFFF - 2;
  }
  else if (one == LOW && two == LOW)
  {
    rs485addr = 0xFFFF - 3;
  }
  return rs485addr;
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

//===============================================================================
//  Initialization
//===============================================================================
void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  speedupPWM();
  analogReference(EXTERNAL);

  // comment out if not debugging
  Serial.begin(115200);

  rs485_address = getrs485address();
  Serial.println(rs485_address);

  Serial1.begin(1000000); // RS485 Serial port
  Serial1.setTimeout(1);  // set timeout to 1 ms

  // assign both bytes of address to first two bytes of outgoing packet
  outgoingShorts[0] = rs485_address;

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
  // valves.setSpeeds(valve_cmd);
}

//===============================================================================
//  Main
//===============================================================================
void loop()
{

  readPressureData(); // expensive... 300-500 us, about half of the loop time I think
  shortToBytes(outgoingShorts, outgoingBytes);
  handleIncomingBytes();

  // Serial.println(pressure_commands[0], DEC);
  // Serial.println(pressure_commands[1], DEC);
  // Serial.println(pressure_commands[2], DEC);
  // Serial.println(pressure_commands[3], DEC);
  // Serial.println(" ");

  for (int i = 0; i < 4; i++)
  {
    valve_cmd[i] = kp * (pressure_commands[i] - pressure_data[i]);
  }

  // Serial.println(valve_cmd[0], DEC);
  // Serial.println(valve_cmd[1], DEC);
  // Serial.println(valve_cmd[2], DEC);
  // Serial.println(valve_cmd[3], DEC);
  // Serial.println(" ");

  // send updated control signals [-400,400]
  valves.setSpeeds(VENT_CMD);
}
