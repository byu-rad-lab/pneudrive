# A4990ValveInterface Arduino Code

## Introduction
This Arduino code is designed for controlling valves using the A4990 motor driver and interfacing with pressure sensors. It provides functionality for reading pressure data and adjusting valve positions accordingly. The code and hardware is designed for each module to be a node on an RS485 bus for high speed and robust communication.

## Valve Command Signals
Valve PWM commands are on the interval [-400, 400]. 
- -400 corresponds to commanding -1 amp (vent), and 400 corresponds to +1 amp (fill).
- The driver boards actively limit current to 0.7 amps, so any command above 0.7 amps will result in 0.7 amps on the hardware.

## Timer Frequencies
- The code makes use of timers for PWM control. It changes the timer frequencies to eliminate annoying hum when controlling the valves.
- Note that changing timer frequencies may affect the built-in delay() and millis() functions.

## Dependencies
- This code relies on the `A4990ValveInterface` library.

## Installation
1. Ensure you have the necessary hardware setup.
2. Install the required Arduino libraries.
3. Upload this code to your Arduino Nano Every.
4. We use the [PlatformIO extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide), so the configuration files are already included. If you are using the Arduino IDE, you will need to manually install the libraries and configure the project. 

## Usage
1. Connect the pressure sensors as specified in the code.
2. Adjust the parameters like `MAX_INPUT`, `saturation_error`, and `kp` to suit your application.
3. Monitor valve commands and pressure data through the Serial monitor if `DEBUG_MODE` is enabled.
4. Use the code to control valves and read pressure data as needed.

## Functions and Their Descriptions

### `byteToShorts(unsigned short *short_array, const byte *byte_array)`
- Converts an array of 8 bytes to an array of 4 shorts.
- Used to decode incoming data.

### `shortToBytes(const unsigned short *short_array, byte *byte_array)`
- Converts an array of 5 shorts to an array of 10 bytes in big-endian format.
- Used to prepare outgoing data.

### `handleIncomingBytes()`
- Handles incoming data from the Serial port.
- Checks for device addresses and processes pressure commands.
- Responds with pressure data if the device address matches.

### `readPressureData()`
- Reads pressure data from analog pins and stores it for use in control.

### `custom_delay(int seconds)`
- Custom delay function for waiting a specified number of seconds since we can't use the built-in delay() function with the modified timer frequencies.

### `getrs485address()`
- Retrieves the RS485 address based on the state of DIP switches.

### `speedupPWM()`
- Configures timer frequencies to eliminate audible noise from valve control.

### `setup()`
- Initializes the hardware and serial communication.
- Sets up timer frequencies and other parameters.

### `loop()`
- The main loop of the program.
- Reads pressure data, processes it, and adjusts valve commands accordingly.

## Author
- Curtis Johnson

## Citation

If you use this code in your work, please cite it as follows:

```bibtex
@software{Johnson2023,
  author = {Curtis C. Johnson},
  title = {A4990ValveInterface Arduino Code},
  year = {2023},
  url = {https://bitbucket.org/byu_rad_lab/byu_pressure_control/src/RS485_validation/arduino_code/arduino_pressure_control/},
  version = {1.0},
}
```


## License
This code is provided under the [License Name], which can be found in the LICENSE file.
