# Build And Flash

## Project-Local Toolchain

This repo can be built with a project-local PlatformIO install:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade platformio
.venv/bin/pio run
```

If PlatformIO is already installed globally:

```bash
pio run
```

## Flash

Confirm the ESP32-S3 serial port first:

```bash
.venv/bin/pio device list
```

Flash to the confirmed port:

```bash
.venv/bin/pio run --target upload --upload-port <PORT>
```

## Serial Monitor

```bash
.venv/bin/pio device monitor --port <PORT> --baud 115200
```

Expected runtime log shape:

```text
DragyDash ESP32 starting.
Scanning for Dragy FD00/DRG advertisements...
Connecting to DRG70-xxxxxx...
Dragy battery=NN%
FD03 challenge NN NN; writing handshake response.
Dragy connected; waiting for FD02 NAV-PVT notifications.
NAV-PVT speed=0.00 km/h sats=8 hAcc=2.0 m fix=3D
```

## Verified Toolchain

The prototype was verified with:

- PlatformIO Core 6.1.19
- Espressif 32 platform 7.0.1
- Arduino-ESP32 framework 2.0.17
- TFT_eSPI 2.5.43
- NimBLE-Arduino 1.4.3

Expected warnings:

- TFT_eSPI warns that DMA is not supported in parallel mode.
- TFT_eSPI warns that touch is not configured.

The firmware does not use touch, and the display is configured for parallel mode.
