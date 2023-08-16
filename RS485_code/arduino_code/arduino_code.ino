/* 
 * Arduino code for serial communication with Odroid using RS485 modules
 */

const char ADDRESS = 'b';

unsigned char pressure_command_bytes[8]; //comes from odroid
unsigned short pressure_command_shorts[4];
// unsigned short pressure_state_shorts[4] = {222, 333, 444, 555}; 
unsigned short pressure_state_shorts[4] = {666, 777, 888, 999};
unsigned char pressure_state_bytes[9] = {ADDRESS, 0x02, 0x9A, 0x03, 0x09, 0x03, 0x78, 0x03, 0xE7}; // sends to odroid

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

//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  Serial1.begin(1000000);  // RS485 Serial port
  pressure_state_bytes[0] = ADDRESS;
  pinMode(LED_BUILTIN, OUTPUT);
}


//===============================================================================
//  Main
//===============================================================================
void loop() {
  if (Serial1.available() == 9) // If data has come in from Odroid
  {
    if (Serial1.read() == ADDRESS) 
    {
      // if address is correct, read in the rest of the message (8 bytes)
      for (int i=0; i<8; i++) {
        pressure_command_bytes[i] = Serial1.read();
      }

      //convert the bytes to shorts to be used by the arduino
      byteToShorts(pressure_command_shorts, pressure_command_bytes);

//      // do something with the pressure commands here
//      if (pressure_command_shorts[0] = 43690)
//      {
//        digitalWrite(LED_BUILTIN, HIGH);
//      }
      
//      // still not sure why, but this is the smallest delay that allows the odroid to receive data
// delay from rs485 boards is 120 us, and packet takes 90 us (120-90 = 30 us), but sending to RX buffer takes about 30 us. so add just a bit.
      delayMicroseconds(5);
//
      // convert the current state into bytes
//      shortToBytes(pressure_state_shorts, pressure_state_bytes);


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