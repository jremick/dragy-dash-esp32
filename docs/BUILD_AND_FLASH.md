# Build And Flash

## Project-Local Toolchain

This repo can be built and unit-tested with a project-local PlatformIO install:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade platformio
.venv/bin/pio test -e native
.venv/bin/pio run -e lilygo_t_display_s3
```

To build the Imperial (`mph`) variant instead of Metric (`km/h`):

```bash
.venv/bin/pio run -e lilygo_t_display_s3_imperial
```

If PlatformIO is already installed globally, run `pio test -e native` and `pio run` directly.

## Flash

Confirm the ESP32-S3 serial port first:

```bash
.venv/bin/pio device list
```

Flash to the confirmed port (substitute `lilygo_t_display_s3_imperial` for `mph`):

```bash
.venv/bin/pio run -e lilygo_t_display_s3 --target upload --upload-port <PORT>
```

Do not flash unless the selected port is clearly the intended LilyGO T-Display-S3.

## Serial Monitor

```bash
.venv/bin/pio device monitor --port <PORT> --baud 115200
```

Expected runtime log shape (serial suffixes are masked by default; telemetry summary is rate-limited to ~1 Hz):

```text
DragyDash ESP32 starting (units=metric).
Scanning for Dragy FD00/DRG advertisements...
Connecting to DRG70-****** (rssi=-54)...
Dragy battery=84%
FD03 handshake response written.
Dragy connected; waiting for FD02 NAV-PVT notifications.
NAV-PVT speed=0.00 km/h sats=8 hAcc=2.0 m fix=3D
```

## Pinned Toolchain

Configured in `platformio.ini`:

- PlatformIO Core 6.x
- `espressif32@8.0.0` (Arduino-ESP32 3.3.12 / ESP-IDF 5.x)
- `bodmer/TFT_eSPI@^2.5.43`
- `h2zero/NimBLE-Arduino@^2.5.2`

Expected library warnings:

- `TFT_eSPI` warns that DMA is not supported in parallel mode.
- `TFT_eSPI` warns that touch is not configured (unused; controls route through GPIO 0 / 14 buttons).
