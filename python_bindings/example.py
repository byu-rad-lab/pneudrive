from pneudrive_py import PressureController

controller = PressureController("/dev/ttyS1", 1)

controller.ping_devices()

controller.set_pressure_command(0, [1, 2, 3, 4])
controller.get_pressure_data(0)
