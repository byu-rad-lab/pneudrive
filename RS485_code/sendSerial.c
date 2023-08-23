#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <wiringSerial.h>

void shortToBytes(unsigned short *short_array, unsigned char *byte_array)
{
  // Function to convert array of 4 shorts to array of 8 bytes (doesn't change first byte because of address byte)
  int shortLength = 4;
  for (int i = 0; i < shortLength; i++)
  {
    int byteIndex = i * 2 + 1;
    unsigned char *bytePtr = (unsigned char *)&short_array[i];
    byte_array[byteIndex] = bytePtr[1];     // Most significant byte
    byte_array[byteIndex + 1] = bytePtr[0]; // Least significant byte
  }
}

void byteToShorts(unsigned short *short_array, unsigned char *byte_array)
{
  // Function to convert array of 8 bytes to array of 4 shorts (states start at i=1)
  unsigned int byteLength = 8;
  for (unsigned int i = 0; i < byteLength; i += 2)
  {
    short_array[i / 2] = ((short)byte_array[i + 1] << 8) | byte_array[i + 2];
  }
}

int main()
{
  int fd; // file descriptor for the serial port

  if ((fd = serialOpen("/dev/ttyS1", 1000000)) < 0)
  {
    fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    return 1;
  }

  if (wiringPiSetup() == -1)
  {
    fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
    return 1;
  }
  // throw this in the constructor.

  while (1)
  {
    serialFlush(fd); // clear the current serial buffer before doing anything with it

    printf("\n\nInput Which arduino to write to: ");
    char arduino_address_write;
    char arduino_address_read;
    unsigned short pressure_cmd[4] = {43690, 43690, 43690, 43690}; // unsigned short size is 2 bytes and can go from 0 to 65535
    unsigned char msg_array[9];
    // assumed to already be done

    unsigned char response_array[8];
    unsigned short arduino_state[4];

    scanf(" %c", &arduino_address_write);
    // printf("Writing to arduino %c", arduino_address);

    // Construct a byte message to send to the arduino
    msg_array[0] = arduino_address_write; // msg_array[0] = joint
    shortToBytes(pressure_cmd, msg_array);

    // printf("\n");
    // for (int i = 0; i < 9; i++) {
    //   printf("%02x ", msg_array[i]);
    // }

    // Write byte message to the arduino
    for (int i = 0; i < 9; i++)
    {
      serialPutchar(fd, msg_array[i]); // write one byte at a time
      //delayMicroseconds(20);
    }
    //delayMicroseconds(500);
    while (serialDataAvail(fd) != 9); // wait for arduino to respond with 9 bytes

    // READ the response from the arduino
    // printf("\nReceived Arduino Response:");
    int pos = 0;
    while (serialDataAvail(fd))
    {
      response_array[pos] = serialGetchar(fd);
      pos++;
      fflush(stdout);
    }

    // Convert the response from the arduino into short array
    if (pos != 9)
    { // didn't read a correct arduino response
      printf("\nGot %i bytes. Expected 9.", pos);
    }

     printf("\n");
     for (int i = 0; i < pos; i++) {
       printf("%02x ", response_array[i]);
     }

    byteToShorts(arduino_state, response_array);
    arduino_address_read = response_array[0];

    printf("\n");
    printf("%c ", arduino_address_read);
    for (int i = 0; i < 4; i++)
    {
      printf("%hu ", arduino_state[i]);
    }

    // printf("\n Array Size: %zu", sizeof(pressure_cmd));
    // printf("\n Address Size: %zu", sizeof(arduino_address));
  }

  return 0;
}
