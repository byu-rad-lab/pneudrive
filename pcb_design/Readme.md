# Files

These PCBs were designed using Autodesk EAGLE. The corresponding project, board, and schematic files are found in this directory, as well as the CAM outputs.

# Places to order parts

For *each* board you will need these things. For each item a link is included where we ordered them.

* 4 ABPDRNN100PGAA5 pressure sensors ([link](https://www.mouser.com/ProductDetail/Honeywell/ABPDRNN100PGAA5?qs=sGAEpiMZZMvhQj7WZhFIAD2P7qVC0dZ7tI11ZYAVyGqQk%2FMhVJdgGw%3D%3D))
* 2 Pololu A4990 Dual Motor Driver Carriers ([link](https://www.pololu.com/product/2137/))
* 2 Pin Dip Switch ([link] (https://www.digikey.com/en/products/detail/cui-devices/DS01-254-S-02BE/11310829))
* 1 Arduino Nano Every ([link] https://store.arduino.cc/usa/nano-every)
	* Note that these also come in packs of 3 or 6, and then they are cheaper ([link] (https://store.arduino.cc/usa/nano-every-pack))
* 7 Phoenix PTSM connectors ([link](https://www.digikey.com/en/products/detail/phoenix-contact/1778625/2625578) to female part, [link](https://www.digikey.com/en/products/detail/phoenix-contact/1778832/2625556) to male part. In previous iterations, we used [these screw terminals](https://www.amazon.com/Simpo-Terminal-Optional-300v10a-Drawing/dp/B018ORUVTU/ref=sr_1_4?dchild=1&keywords=2.54%2Bmm%2Bscrew%2Bterminal&qid=1587763307&s=industrial&sr=1-4&th=1).)
* 4 12 kOhm resistors ([link](https://www.amazon.com/EDGELEC-Resistor-Tolerance-Resistance-Optional/dp/B07HDGQRSR/ref=sr_1_2?dchild=1&keywords=12k+resistor&qid=1587767472&sr=8-2))
* 2 4.7 kOhm resistors ([link](https://www.amazon.com/4-7-kOhm-Resistor/s?k=4.7+kOhm+Resistor))
* ~62 Female header pins (really only 58, but each cut destroys one) ([link](https://www.amazon.com/Qunqi-2-54mm-Straight-Connector-Arduino/dp/B07CGGSDWF/ref=sr_1_3?dchild=1&keywords=female+header+pins&qid=1587769954&sr=8-3))
* 8 100 nF capacitors ([link](https://www.amazon.com/Gikfun-Ceramic-Capacitor-Arduino-100pcs/dp/B00RT02YIU/ref=sr_1_9?dchild=1&keywords=100nf+capacitor&qid=1587767206&sr=8-9))
* 4 1 nF capacitors (We got away without this capacitor in the prototype...I may try that again...)

And for *each* BBB cape, you'll need:

* 1 Logic Level Shifter, 4-Channel, Bidirectional ([link] https://www.pololu.com/product/2595)
* 2 2.2k resistors
* 1 Phoenix PTSM connector ([link](https://www.digikey.com/en/products/detail/phoenix-contact/1778625/2625578) to female part, [link](https://www.digikey.com/en/products/detail/phoenix-contact/1778832/2625556) to male part. In previous iterations, we used [these screw terminals](https://www.amazon.com/Simpo-Terminal-Optional-300v10a-Drawing/dp/B018ORUVTU/ref=sr_1_4?dchild=1&keywords=2.54%2Bmm%2Bscrew%2Bterminal&qid=1587763307&s=industrial&sr=1-4&th=1).)


You will also need wires for:

* I2C communication. We used 8128T1 from [here](https://www.mcmaster.com/shielded-wire) because it is small gauge (24 AWG) and double shielded.
* 12 V power and ground.
* Splicing connections are useful for getting 12V to each arudino board in a quick and clean way ([link] (https://www.amazon.com/dp/B07XMJ5KTY/ref=sspa_dk_detail_0?spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUFPUFdLM1RZU1g4M0kmZW5jcnlwdGVkSWQ9QTAzODQzNDEyU1RMUVlGNVgwM1VNJmVuY3J5cHRlZEFkSWQ9QTAwNTA5MTUyWERMTFQ3TzBUUDNIJndpZGdldE5hbWU9c3BfZGV0YWlsMiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU&th=1))

Apart from the 5V power supply which came with your BBB, you will need a power supply capable of 7-21 V DC and however many amps your system can pull (4 A for each board). We choose to use a 12 V 15 A one ([link] (https://www.amazon.com/AVAWO-Switching-Transformer-Regulated-Computer/dp/B0146IAXYO/ref=sr_1_3?dchild=1&keywords=24+volt+power+supply&qid=1587765261&sr=8-3))

# Populating the board

The image below shows the top-down view of the board with some added annotations. The yellow highlighted sections represent the areas where we soldered headers to the board (we plug the Arduino and both motor controllers into headers rather than soldering them directly to the board).
Values and placement for resistors and capacitors are also shown on the image below. Pressure sensors should be soldered in the areas labeled PS1 - PS4 along the top edge of the board. The connectors for power, I2C, and valves are located along the opposite edge.

![PCB Labels](/pcb_top.png "PCB Silkscreen and Values")

# Things to consider for a future revision of the board

* Find a robust way to plug the pressure sensors into the board. Right now they're soldered directly to the board, but it would be convenient to be able to unplug and replace them.
* The Arduino Pro mini can take 12 V power and regulate it down to 5V on the VCC pin. This would eliminate the need to run separate 5V and 12V wires.
* Update the silkscreen for more complete labeling