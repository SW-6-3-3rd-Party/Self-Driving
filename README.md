# Self-Driving Project

## Current ToF Wiring

The front ToF sensor is configured for I2C mode.

- ToF VIN -> Raspberry Pi 3.3V or sensor-rated VIN
- ToF GND -> Raspberry Pi GND
- ToF SCL -> Raspberry Pi GPIO 3, SCL1, physical pin 5
- ToF SOA/SDA -> Raspberry Pi GPIO 2, SDA1, physical pin 3
- ToF GPIO1 -> leave unconnected
- ToF XSHUT -> leave unconnected, or pull up to VIN if the module does not start
