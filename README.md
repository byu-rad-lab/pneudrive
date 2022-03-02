## What is this repository?

This repository contains all of the designs, code, and instructions needed to create a low level pressure controller designed for pneumatically actuated robots. This pressure controller is designed to use relatively inexpensive off-the-shelf components and to favor simplicity in an effort to make it easy to understand and adapt for different applications.

The pressure control system consists of one BeagleBone Black (BBB) acting as the ROS master and communicating over I2C with one or more Arduinos. Each Arduino performs high-rate pressure control of up to 4 chambers and transmits pressure data back to the ROS master. A high level diagram of the pressure control system can be seen in the figure below:

![Alt](/pressure_control_system_diagram.png "Pressure Control System Diagram")

---

## How is it organized

This repository is a ROS package designed to be placed in a ROS workspace on the BeagleBone used for pressure control. There are however several other directories not necessary for a ROS package:

* **pcb_design** - this directory contains all of the wiring diagrams and PCB schematics. Instructions for the population of the PCB as well as wiring the entire pressure controller are found here.
* **beaglebone_setup** - this directory contains all of instructions and files necessary to setup a beaglebone for use as the ROS master in a pressure control system.
* **arduino_code** - this directory contains the instructions and code needed to program the arduinos to act as high rate pressure controllers.

---

## Steps to create a pressure control system

1. Use the instructions and schematics in the **pcb_design** directory to order and assemble the necessary components to construct the populated PCB.
2. Use the instructions and files in the **beaglebone_setup** directory to setup the BeagleBone as the ROS master
3. Use the instructions and files in the **arduino_code** directory to program each arduino as a high-rate pressure controller

Once all of the PCBs have been populated and wired, the BBB has been setup, and the Arduinos have been programmed, the system should perform pressure control as soon as power is applied. As soon as power is applied, the Arduinos begin performing pressure control (to the default pressure of 0 psi). 

Once the BBB boots up it you can ssh into it with 

```ssh ubuntu@192.168.0.xxx```

where xxx is the chosen IP address you chose when setting up the beaglebone. Note the boot time is currently pretty slow. It takes 2-3 minutes to be ready to ssh into it. This is a factor that can be improved in future work. 

Once you ssh into the BBB, you can launch the pressure control ROS node (along with a roscore) using 

``` roslaunch byu_pressure_control i2c_pressure.launch ```

This creates a ROS node called "PressureController". This launch file looks in the /config directory for a .yaml file describing the hardware configuration of whatever platform you are using. The node creates publishers and subscribers for every joint (with it cooresponding I2C address) that is specified in the yaml file. If the BBB cannot find all of the specified addresses, the node throw an error and shut down. If this happens, here are some common fixes:

* Make sure your wires are all connected securely.
* Make sure that each pressure control board dip switch is set to the proper I2C address. The dip switch positions are printed on the pcb and they should match what is specified in the yaml file. For example, if I have a 3 joint arm, I would make sure that the first joint is set to 0xa then the next as 0xb and the last as 0xc, with those same addresses specified in the yaml file. 
* Make sure you didn't flip the SCL and SDA wires when wiring up the connectors. We chose to use crimp-less connectors for ease of use, but this does make flipping wires easier if you aren't being careful. 

When the node launches, it will print out all of the joints it detected and start the node. The code then loops through as fast as it can to transfer data from the I2C bus to ROS and vice versa. Obviously, the more nodes you add on, the less frequently each node will be updated. Currently, we are using the fastest I2C mode supported on the BBB which is [FAST Mode](https://en.wikipedia.org/wiki/I%C2%B2C#:~:text=system%20of%20boards.-,I2C%20modes,-Mode%5B12%5D) which has a data rate of 400 kbps.

There are lots more details than this, but this analysis provides a ballpark estimate of overall data rates as a function of number of joints. The basic I2C message looks like this:

![Alt](/i2c_message.png "I2C Basics")

The arduino code and the BB code both have a constant called BYTES_PER_PRESSURE defined. By default this is set to 2, meaning 2 bytes (or 16 bits) are used to represent a single pressure reading. This can also be set to 1 (8 bit precision) which will speed up data at the cost of a courser resolution. 


#### Sidenote on I2C Messages
You'll notice that the beaglebone code refers to both device addresses as well as register addresses. BOTH addresses are needed when interfacing the BB with a non-microcontroller device such as an IMU where data is simply written to internal registers (see diagram below). Here's a good [video](https://www.youtube.com/watch?v=8C2zk6B-eLU&t=2847s) of this in action. In our case, we can use variables and the arduino Wire library, so we don't need the register address. So our messages actually look like the previous diagram above (i.e. no register address). This note is just here to avoid any confusion when looking through the code. The readRegisters() and writeRegisters() methods of each i2c device still accept a register address to preserve the capability to interface with non-arduino i2c devices, but the buffer of bits actually sent down the wire only contains data, not any register addresses. 

![Alt](/bigger_i2c_message.png "I2C Bigger")

## Things to consider in future versions

* The current PCB connects the Arduino the the fault pins on the motor driver boards, but we do nothing with it. You may be able to do something like light up an LED for specific errors.
* Look into boot time improvements for BBB.
* The dip switch we selected only has 2 pins, which means there are 4 available hardware settable I2C addresses. You can easily set the I2C address manually on the arduino firmware and ignore the dip switch if you want to have more than 4 pressure control devices on the same bus, but maybe it would be worth trying a 3 pin dip switch (for 8 total addresses) and rearranging some pins on the pressure pcb. We will likely never have more than 4 joints on one arm though. 

