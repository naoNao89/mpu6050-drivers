# ESP32-C3 MPU6050 firmware

Board-specific bring-up firmware for the connected ESP32-C3 + MPU6050 setup.

Build, flash, and monitor from the repository root using the root `Makefile` or `run.sh`, for example:

```sh
make build
./run.sh
```

Check the firmware without building host-only workspace members for the embedded
target:

```sh
make check-firmware
```
