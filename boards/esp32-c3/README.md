# ESP32-C3 MPU6050 firmware

Board-specific bring-up firmware for the connected ESP32-C3 + MPU6050 setup.

Reference wiring for this board profile:

| GY-521/MPU6050 | ESP32-C3 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SCL | GPIO0 |
| SDA | GPIO1 |
| XDA | GPIO3 |
| XCL | GPIO4 |
| AD0 | GPIO5 |
| INT | GPIO6 |

Build, flash, and monitor from the repository root using the root `Makefile` or `run.sh`, for example:

```sh
make build
./run.sh
```
