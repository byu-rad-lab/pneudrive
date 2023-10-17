To build the bindings, you will just need to navigate to the python_bindings directory and build the bindings manually with cmake:

``` console
cd python_bindings
mkdir build
cd build
cmake ..
make
```

Once they are built, an importable python module called pneudrive_py will show up as a shared library in the build directory. You can import this module into any python script and use it to communicate with the pressure controllers. Here is a simple example:

``` python
import numpy as np
from pneudrive_py import PressureController

uart_port = '/dev/ttyS1'
num_devices = 4
my_controller = PressureController(uart_port, num_devices)

#check communication with all expected devices
my_controller.ping_devices()

pressure_cmd = np.array([1,2,3,4])
for i in range(num_devices):
    my_controller.set_pressure_commands(i, pressure_cmd)
    data = my_controller.get_pressure_data(i)

```

Setting the pressure commands for the ith device using ``` set_pressure_commands()``` initiates a call and repsonse transmission. The SBC controller will send the commands to the embedded device and the embedded device will respond with pressure measurements, accessible in Python via ``` get_pressure_data()```. 

Note that this means that successive calls to ```get_pressure_data()``` for the same device will return the same data. If you want to get new data, you will need to call ```set_pressure_commands()``` again.