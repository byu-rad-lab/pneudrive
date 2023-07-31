#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <wiringSerial.h>

void shortToBytes(unsigned short *value, unsigned char *byte_array) {
  // Function to convert array of 4 shorts to array of 8 bytes (doesn't change first byte because of address byte)
  int shortLength = 4;
  for (int i=0; i<shortLength; i++) {
    int byteIndex = i*2+1;

    unsigned char *bytePtr = (unsigned char *)&value[i];
    byte_array[byteIndex] = bytePtr[1]; // Most significant byte
    byte_array[byteIndex+1] = bytePtr[0]; // Least significant byte
  }
}

void byteToShorts(unsigned short *short_array, unsigned char *byte_array) {
  //Function to convert array of 8 bytes to array of 4 shorts
  unsigned int byteLength = 8;
  for (unsigned int i=0; i<byteLength; i+=2) {
    short_array[i/2] = ((short)byte_array[i] << 8) | byte_array[i+1];
  }
}

int main () {
  int fd; // file descriptor for the serial port

  if ((fd = serialOpen("/dev/ttyS1", 1000000)) < 0) {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror(errno));
    return 1;
  }

  if (wiringPiSetup() == -1) {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror(errno));
    return 1;
  }

  while(1) {
    serialFlush(fd); // clear the current serial buffer before doing anything with it

    printf("\n\nInput Which arduino to write to: ");
    char arduino_address;
    unsigned short pressure_cmd[4] = {123, 456, 789, 1011}; // unsigned short size is 2 bytes and can go from 0 to 65535
    unsigned char msg_array[9];

    unsigned char response_array[8];
    unsigned short arduino_state[4];

    scanf(" %c", &arduino_address);
    printf("\nWriting to arduino %c", arduino_address);

    msg_array[0] = arduino_address;
    shortToBytes(pressure_cmd, msg_array);

    printf("\n");
    for (int i = 0; i < 9; i++) {
      printf("%02x ", msg_array[i]);
    }

    // write msg_array to arduino
    for (int i=0; i<9; i++) {
      serialPutchar(fd, msg_array[i]); //write one byte at a time
    }

    delay(10);
    printf("\nReceived Arduino Response:");
    int pos = 0;
    while (serialDataAvail(fd)) {
      response_array[pos] = serialGetchar(fd);
      pos++;
      fflush(stdout);
    }

    if (pos != 8) { // didn't read a correct arduino response
      printf("\nGot %i bytes. Expected 8.", pos);
    }

    printf("\n");
    for (int i = 0; i < 8; i++) {
      printf("%02x ", response_array[i]);
    }

    byteToShorts(arduino_state, response_array);
    printf("\n");
    for (int i = 0; i < 4; i++) {
      printf("%hu ", arduino_state[i]);
    }


    // printf("\n Array Size: %zu", sizeof(pressure_cmd));
    // printf("\n Address Size: %zu", sizeof(arduino_address));
  }

  return 0 ;
}