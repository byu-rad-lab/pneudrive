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

short valve_cmd[4] = { 0, 0, 0, 0 };
short VENT_CMD[4] = { -400, -400, -400, -400 };
short FILL_CMD[4] = { 400, 400, 400, 400 };

const unsigned short MAX_INPUT = 400;
unsigned short saturation_error = 45; //tuned with Ziegle-Nichols method, Ku ~ 35 is oscillations.
unsigned short kp = MAX_INPUT / saturation_error;
unsigned short kd = 100; // 400 too high, acting on noise

#define PRESCALER 32
#define ONE_SECOND 32000
#define BYTES_IN_PACKET 10
#define DEBUG_MODE false

byte outgoingBytes[BYTES_IN_PACKET] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte incomingDataBytes[BYTES_IN_PACKET - 2] = { 0, 0, 0, 0, 0, 0, 0, 0 };

unsigned short outgoingShorts[BYTES_IN_PACKET / 2] = { 0, 0, 0, 0, 0 };

unsigned short rs485_address = 0x0000;
unsigned short pressure_commands[4] = { 0, 0, 0, 0 };
unsigned short pressure_data[4] = { 0, 0, 0, 0 };
unsigned short prev_pressure_data[4] = { 0, 0, 0, 0 };
unsigned long alpha = 210;

// Function to convert array of 8 bytes to array of 4 shorts
void byteToShorts(unsigned short* short_array, const byte* byte_array)
{
  unsigned int byteLength = 8;
  for (size_t i = 0; i < byteLength; i += 2)
  {
    int byteIndex = i;
    short_array[i / 2] = ((short)byte_array[byteIndex] << 8) | byte_array[byteIndex + 1];
  }

  if (DEBUG_MODE)
  {
    Serial.print("Incoming shorts: ");
    for (int i = 0; i < 4; i++)
    {
      Serial.print(short_array[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Function to convert array of 5 shorts to array of 10 bytes, but in big endian
void shortToBytes(const unsigned short* short_array, byte* byte_array)
{
  int shortLength = 5;
  for (size_t i = 0; i < shortLength; i++)
  {
    int byteIndex = i * 2;
    unsigned char* bytePtr = (unsigned char*)&short_array[i];

    // Convert from little endian to big endian
    byte_array[byteIndex] = bytePtr[1];     // LSB
    byte_array[byteIndex + 1] = bytePtr[0]; // MSB
  }

  if (DEBUG_MODE)
  {
    Serial.print("Outgoing bytes: ");
    for (int i = 0; i < 10; i++)
    {
      Serial.print(byte_array[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void handleIncomingBytes()
{
  byte firstByte = 0;
  while (Serial1.available() > 0)
  {
    if (DEBUG_MODE)
    {
      Serial.print("Available bytes: ");
      Serial.println(Serial1.available(), DEC);
    }
    byte secondByte = Serial1.read();

    if (DEBUG_MODE)
    {
      Serial.print("First byte: ");
      Serial.println(firstByte, DEC);
      Serial.print("Second byte: ");
      Serial.println(secondByte, DEC);
    }

    unsigned short short1 = (firstByte << 8) | secondByte;

    if (short1 == rs485_address)
    {
      if (DEBUG_MODE)
      {
        Serial.println("Address found!");
      }

      size_t numBytesRead = Serial1.readBytes(incomingDataBytes, BYTES_IN_PACKET - 2);

      if (DEBUG_MODE)
      {
        Serial.print("Incoming bytes: ");
        for (int i = 0; i < 8; i++)
        {
          Serial.print(incomingDataBytes[i], DEC);
          Serial.print(" ");
        }
        Serial.println();
      }

      if (numBytesRead == 8)
      {
        Serial1.flush();
        if (Serial1.availableForWrite() == SERIAL_TX_BUFFER_SIZE - 1)
        {
          size_t numBytesWritten = Serial1.write(outgoingBytes, BYTES_IN_PACKET);
          if (DEBUG_MODE)
          {
            Serial.print("Outgoing bytes written: ");
            Serial.println(numBytesWritten, DEC);
          }
        }
        byteToShorts(pressure_commands, incomingDataBytes);

        firstByte = 0;
      }
      else
      {
        // Serial.print("Timeout. Bytes read: ");
        // Serial.println(numBytesRead, DEC);
      }
    }
    else
    {
      if (DEBUG_MODE)
      {
        Serial.println("Address not found.");
      }

      firstByte = secondByte;
    }
  }
}

void readPressureData()
{
  for (int i = 0; i < 4; i++)
  {
    //update delayed vars before reading new data
    prev_pressure_data[i] = pressure_data[i];

    // low pass filter with fixed point math for speed
    unsigned long temp = alpha * pressure_data[i] + (256 - alpha) * analogRead(A0 + i); // max value is 1023*256 = 261888, need a long to prevent overflow
    pressure_data[i] = static_cast<unsigned short>(temp / 256);                         // divide by 256, back to short range
    outgoingShorts[i + 1] = pressure_data[i];
  }

  if (DEBUG_MODE)
  {
    Serial.print("Pressure data: ");
    for (int i = 0; i < 4; i++)
    {
      Serial.print(pressure_data[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void custom_delay(int seconds)
{
  for (int s = 0; s < seconds; s++)
  {
    delay(ONE_SECOND);
  }
}

int getrs485address()
{
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
  /* see section 20.5.1 of http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-4809-Data-Sheet-DS40002173A.pdf
    for this register settings. Note that by default, fclk_per = 62.4 kHz.
    Humans can hear 20 Hz to 20 kHz, so a 31.2kHz freq should be out of hearing range.
    Also see section 5.4.1 for syntax used here.
    | is "bitwise or" commonly used to set multiple bits at once in a register.
    TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm --> 0,0,1,0 OR 0,0,0,1 = 0,0,1,1

    READ TIMER FREQUENCY NOTE AT TOP
*/
  cli(); // Disable Interrupts

  // Set timer frequencies to 31.2 kHz
  TCB0.CTRLA = (TCB_CLKSEL_CLKDIV2_gc) | (TCB_ENABLE_bm);
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;

  sei(); // Enable Interrupts
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  speedupPWM();
  analogReference(EXTERNAL);

  rs485_address = getrs485address();

  if (DEBUG_MODE)
  {
    Serial.begin(115200);
    Serial.print("Device address: ");
    Serial.println(rs485_address, DEC);
  }

  Serial1.begin(1000000); // RS485 Serial port
  Serial1.setTimeout(1);

  outgoingShorts[0] = rs485_address;

  valves.setSpeeds(VENT_CMD);

  custom_delay(5);
}

void loop()
{
  readPressureData();
  shortToBytes(outgoingShorts, outgoingBytes);
  handleIncomingBytes();

  for (int i = 0; i < 4; i++)
  {
    int proportional = kp * (pressure_commands[i] - pressure_data[i]);
    int derivative = kd * (pressure_data[i] - prev_pressure_data[i]);
    valve_cmd[i] = proportional - derivative;
  }

  valves.setSpeeds(valve_cmd);

  if (DEBUG_MODE)
  {
    Serial.print("Valve commands: ");
    for (int i = 0; i < 4; i++)
    {
      Serial.print(valve_cmd[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
  }
}
