# Files

These PCBs were designed using Autodesk EAGLE. The corresponding project, board, and schematic files are found in this directory, as well as the CAM outputs.

We used OSHPARK to print out PCBs. For convenience, their design rules are also included in this repo. To use them, navigate to 'Design Rules' in Eagle and click load to select the .dru file. This will load OSHPARK's manufacturing rules automatically for design rule checking.

We have also created a custom part library which includes footprints, schematics, and 3d models for each major component used in this design. To use the library, simply add the filepath of the .lbr file to Eagle's search path in the Control Panel. For more info see [here](https://www.instructables.com/Adding-a-Library-to-Eagle-CAD/).

# Specifications and Design Notes
## I2C Design
The I2C bus on this pcb has both an IN and OUT port. The OUT port is useful for daisy chaining multiple of these pcbs, but is not required. The arduino communicates on a 5V logic level, which is incompatible with the 3.3V logic of the BBB. To remedy this, we use a bi-directional logic level shifter to make sure voltages are at the right levels for each device. The shifter is used once on the BBB cape and outputs 5V to any arudino board down the line. The logic shifter uses 10k pullup resistors on the SDA and SCL lines. According to [this](https://www.ti.com/lit/an/slva689/slva689.pdf?ts=1612978067135&ref_url=https%253A%252F%252Fwww.google.com%252F) resource, 10k is too high for our I2C line. 

The max for I2C fast mode is 400 pF according to Table 1. Since we don't have that many devices and the wires aren't super long, we assumed a bus capacitance of about 150pF. This combined with the operating voltage of 5V on fast mode gives a minimum resistance value of about 1.5k (from Fig 2) and a max resistance of about 2.5k 9 (from Fig 3). In order to get an equivalent resistance in this range with the 10k resistors in the logic shifter, we add a 2.2k resistor in parallel to get an equivalent pullup resistance of 1.8k. We measured the performance of the I2C bus with these resistors in place and observed about 380-390Hz bit rate (the 10k resistors alone work, but the bit rate was ~340Hz and the voltage level was barely reaching high). The closer the equivalent resistance is to the minimum resistance, the faster the line can transfer data at the cost of higher power consumption. 

## PCB Design Notes

* Trace widths on the board are mostly the default that eagle uses with a few exceptions. The 12V line is 80mils wide and the 5V line is 30 mils wide. The trace for the 12V line was made as as large as possible because it is a high current line. Each 2 phase driver board can deliver 0.7 amps/phase, which gives a total continuous current of up to 2.8A on each board. The driver boards can go up to 0.9A/phase, but not for long. Using [this](https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-pcb-trace-width) trace width calculator tool (with 2.8A, 1oz/ft^2, and an ambient temp of 25C) gives us an acceptably low temperature rise (5-10C) for an external trace.

* There are two ground planes in this PCB. The top plane is for the signal ground and the bottom plane is the motor power ground. This was done in an attempt to minimize noise from the motors on the analog pressure sensors. These ground are connected through the GND pins on the driver boards so that the both ground planes are connected to each other when it is running. But by only connecting the ground planes through the driver boards, it encourages any noisy currents to follow the path of least impedance (the ground plane) back to source instead of going through the signal ground. We used [this](https://www.nxp.com/docs/en/application-note/AN1259.pdf) and [this](https://electronics.stackexchange.com/questions/112508/ground-vs-power-ground) as resources as to why we chose to do this. We also did our best to separate the two areas of the board (i.e. the drivers from the pressure sensors) as much as possible without making the board too big. 

# PCB Schematic
![Schematic](schematic_v5.png)

# Places to order parts

For *each* board you will need these things. For each item a link is included where we ordered them. The total price per board (including printing the pcb) is about $150. Note that there are farily significant bulk discounts for many of these parts. 

* 4 ABPDRNN100PGAA5 pressure sensors ([link](https://www.mouser.com/ProductDetail/Honeywell/ABPDRNN100PGAA5?qs=sGAEpiMZZMvhQj7WZhFIAD2P7qVC0dZ7tI11ZYAVyGqQk%2FMhVJdgGw%3D%3D))
* 2 Pololu A4990 Dual Motor Driver Carriers ([link](https://www.pololu.com/product/2137/))
* 2 Pin Dip Switch ([link](https://www.digikey.com/en/products/detail/cui-devices/DS01-254-S-02BE/11310829))
* 1 Arduino Nano Every ([link](https://store.arduino.cc/usa/nano-every))
	* Note that these also come in packs of 3 or 6, and then they are cheaper ([link](https://store.arduino.cc/usa/nano-every-pack))
* 7 Phoenix PTSM connectors ([link](https://www.digikey.com/en/products/detail/phoenix-contact/1778625/2625578) to female part, [link](https://www.digikey.com/en/products/detail/phoenix-contact/1778832/2625556) to male part. In previous iterations, we used [these screw terminals](https://www.amazon.com/Simpo-Terminal-Optional-300v10a-Drawing/dp/B018ORUVTU/ref=sr_1_4?dchild=1&keywords=2.54%2Bmm%2Bscrew%2Bterminal&qid=1587763307&s=industrial&sr=1-4&th=1).)
* 4 12 kOhm resistors ([link](https://www.amazon.com/EDGELEC-Resistor-Tolerance-Resistance-Optional/dp/B07HDGQRSR/ref=sr_1_2?dchild=1&keywords=12k+resistor&qid=1587767472&sr=8-2))
* 2 4.7 kOhm resistors ([link](https://www.amazon.com/4-7-kOhm-Resistor/s?k=4.7+kOhm+Resistor))
* ~62 Female header pins (really only 58, but each cut destroys one) ([link](https://www.amazon.com/Qunqi-2-54mm-Straight-Connector-Arduino/dp/B07CGGSDWF/ref=sr_1_3?dchild=1&keywords=female+header+pins&qid=1587769954&sr=8-3))
* 8 100 nF capacitors ([link](https://www.amazon.com/Gikfun-Ceramic-Capacitor-Arduino-100pcs/dp/B00RT02YIU/ref=sr_1_9?dchild=1&keywords=100nf+capacitor&qid=1587767206&sr=8-9))
* 4 1 nF capacitors (We got away without this capacitor in the prototype...I may try that again...)

And for *each* BBB cape, you'll need:

* 1 Logic Level Shifter, 4-Channel, Bidirectional ([link](https://www.pololu.com/product/2595))
* 2 2.2k resistors
* 1 Phoenix PTSM connector ([link](https://www.digikey.com/en/products/detail/phoenix-contact/1778625/2625578) to female part, [link](https://www.digikey.com/en/products/detail/phoenix-contact/1778832/2625556) to male part. In previous iterations, we used [these screw terminals](https://www.amazon.com/Simpo-Terminal-Optional-300v10a-Drawing/dp/B018ORUVTU/ref=sr_1_4?dchild=1&keywords=2.54%2Bmm%2Bscrew%2Bterminal&qid=1587763307&s=industrial&sr=1-4&th=1).)
* BBB protocape ([link](https://www.adafruit.com/product/572)). This is what we are using for now. The plan is to eventually design a pcb to replace this.


Additional random stuff:

* I2C communication wires. We used this[here](https://www.mcmaster.com/8128T1/) because it is double shielded and rated for up to 35 Mbps data rates (we are at 400kbps).
* 12v DC power supply. Rated for at least 12A continuous. (each board can pull max 2.8 amps continuously, up to 3.6 amps briefly) ([link](https://www.amazon.com/AVAWO-Switching-Transformer-Regulated-Computer/dp/B0146IAXYO/ref=sr_1_3?dchild=1&keywords=24+volt+power+supply&qid=1587765261&sr=8-3))
* 12 V power and ground wire. (20-26 AWG wire to fit in Phoenix connectors). Rated for at least 10 amps. We used the black and red versions here ([link](https://www.mcmaster.com/8054T14/)
* Splicing connections are useful for getting 12V to each arudino board in a quick and clean way ([link](https://www.amazon.com/dp/B07XMJ5KTY/ref=sspa_dk_detail_0?spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUFPUFdLM1RZU1g4M0kmZW5jcnlwdGVkSWQ9QTAzODQzNDEyU1RMUVlGNVgwM1VNJmVuY3J5cHRlZEFkSWQ9QTAwNTA5MTUyWERMTFQ3TzBUUDNIJndpZGdldE5hbWU9c3BfZGV0YWlsMiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU&th=1))
* Digitally controlled power strip for controlling power to arduinos from BBB on e-stop. ([link](https://www.sparkfun.com/products/14236))
* Plastic tubing that fits pressure sensors (2mm ID, rated for at least 85 psi). We use [this](https://www.mcmaster.com/8014N13/).
* Barbed fittings that fit tubing (2mm, rated for up to 100 psi). We use [this](https://www.mcmaster.com/6220N78/).


# Board and CAD File

Note that the female connectors are prone to bending when plugging things in. So be careful with them. It is a good idea to put a dab of hot glue under each connector when populating the board to make them more robust. Just make sure to not cover any conductive surfaces with the glue.

See [here](https://a360.co/2NUtMc7) to see the 3d version of the board. You can download a step file too if you want to design a case, for example.

# Things to consider for a future revision of the board

* Find a robust way to plug the pressure sensors into the board. Right now they're soldered directly to the board, but it would be convenient to be able to unplug and replace them.
* Possibly use a different connector. The female connectors are prone to bending if you're not careful plugging them in. We really like the male spring loaded connectors though because they don't require crimping. Currently, we put a dab of hot glue under the female connector to keep it more secure on the board.
* Do something with the EF pins on the driver boards. Right now they are connected to the arduino, but not doing anything with them. 
* We are using shielded cable for the i2c bus. Technically, the cable shield should be grounded somewhere. It isn't currently. Could add a screw terminal to the board to ground the bare ground wire possibly.