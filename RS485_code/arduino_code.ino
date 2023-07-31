/* 
 * Arduino code for serial communication with Odroid using RS485 modules
 */

const char ADDRESS = 'a';

unsigned char pressure_command_bytes[8];
unsigned short pressure_command_shorts[4];
// unsigned char pressure_state_bytes[8] = {0x04, 0x4D, 0x02, 0x8E, 0x03, 0xDB, 0x01, 0x41};
//                                       // 1101 654, 987, 321 just for an example

unsigned char pressure_state_bytes[8] = {0x01, 0x41, 0x02, 0x8E, 0x03,0xDB, 0x04, 0x4D,};
                                      // 321, 654, 987, 1101 just for an example

void byteToShorts(unsigned short *short_array, unsigned char *byte_array) {
  //Function to convert array of 8 bytes to array of 4 shorts
  unsigned int byteLength = 8;
  for (unsigned int i=0; i<byteLength; i+=2) {
    short_array[i/2] = ((short)byte_array[i] << 8) | byte_array[i+1];
  }
}

//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  Serial.begin(9600);
  Serial1.begin(1000000);  // RS485 Serial port
}
//===============================================================================
//  Main
//===============================================================================
void loop() {
  if (Serial1.available()) // If data has come in from Odroid
  {
    // Serial.print(Serial1.available());
    // Serial.print(" ");
    if (Serial1.read() == ADDRESS) { // check if the message is for this arduino
      for (int i=0; i<8;i++) { // read in the rest of the message (8 bytes)
        // delay(5);
        pressure_command_bytes[i] = Serial1.read();
        // Serial.print(pressure_command_bytes[i], HEX);
        // Serial.print(" ");
      }
      // Serial.println(" ");

      byteToShorts(pressure_command_shorts, pressure_command_bytes); //convert the bytes to shorts to be used by the arduino

      for (int i=0; i<8; i++) {
        Serial.print(pressure_state_bytes[i], HEX);
        Serial.print(" ");
      }
      Serial.println(" ");

      delay(100); // the delay is necessary to get the correct input back to the odroid, it can probably be replaced with code that the arduino has to execute on the real arm??

      // while(Serial1.availableForWrite() < 8) {}
      // Serial1.write(pressure_state_bytes, 8);    // Send the current pressure state back to Odroid
      for (int i=0; i<8; i++) {
        Serial1.write(pressure_state_bytes[i]);
      }
    }
  }
}