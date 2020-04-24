# Places to order parts

For *each* board you will need:

* 4 ABPDRNN100PGAA5 pressure sensors [order here](https://www.mouser.com/ProductDetail/Honeywell/ABPDRNN100PGAA5?qs=sGAEpiMZZMvhQj7WZhFIAD2P7qVC0dZ7tI11ZYAVyGqQk%2FMhVJdgGw%3D%3D)
* 2 Pololu A4990 Dual Motor Driver Carriers [order here](https://www.pololu.com/product/2137/)
* 1 Arduino Pro Mini [order here](https://www.sparkfun.com/products/11113) or anywhere else. Just make sure pin footprints match.
* 7 screw terminals [order here](https://www.amazon.com/Simpo-Terminal-Optional-300v10a-Drawing/dp/B018ORUVTU/ref=sr_1_4?dchild=1&keywords=2.54%2Bmm%2Bscrew%2Bterminal&qid=1587763307&s=industrial&sr=1-4&th=1). They must have 2.54 mm pitch in order to fit in the PCB. They also must accomodate the different wire gauges needed for communication and 32 V power (24 AWG and 18 AWG)
* 4 12 kOhm resistors
* 8 100 nF capacitors
* 4 1 nF capacitors
* 60 Female header pins (really only 56, but each cut destroys one)


You will also need wires for:

* I2C communication. We used 8128T1 from [here](https://www.mcmaster.com/shielded-wire) because it is small gauge (24 AWG) and double shielded.
* 5V power and ground. **Note** I don't know what we're going to use yet
* 32 V power and ground. **Note** I don't know what we're going to use yet

Apart from the 5V power supply which came with your BBB, you will need a power supply capable of 9-32 V DC and however many amps your system can pull (4 A for each board). We choose to use a 24 V 15 A one like [this](https://www.amazon.com/AVAWO-Switching-Transformer-Regulated-Computer/dp/B0146IAXYO/ref=sr_1_3?dchild=1&keywords=24+volt+power+supply&qid=1587765261&sr=8-3)
