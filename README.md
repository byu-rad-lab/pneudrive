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

Once all of the PCBs have been populated and wired, the BBB has been setup, and the Arduinos have been programmed, the system should perform pressure control as soon as power is applied. As soon as power is applied, the Arduinos begin performing pressure control (to the default pressure of 0 psi). Once the BBB boots up it creates a ROS node called "PressureController". This node creates publishers and subscribers for every node that it detects on the I2C bus when it boots up. It then cycles through as fast as it can to transfer data from the I2C bus to ROS and vice versa. Obviously, the more nodes you add on, the less frequently each node will be updated.

