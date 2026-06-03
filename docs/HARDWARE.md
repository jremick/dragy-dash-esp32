# Hardware Assumptions

## Board

Target board:

- LilyGO T-Display-S3
- ESP32-S3
- ST7789 170x320 display
- 8-bit parallel display bus

PlatformIO board ID:

```ini
board = lilygo-t-display-s3
```

## Pins

The firmware assumes the common LilyGO T-Display-S3 factory pin map:

- LCD power: GPIO 15
- LCD backlight: GPIO 38
- Left/page-back button: GPIO 0
- Right/page-forward button: GPIO 14

Display pins are defined in `include/TFT_eSPI_Setup.h`.

## Orientation

The screen is configured with:

```cpp
tft.setRotation(3);
```

This is a 180-degree flip relative to `setRotation(1)` and is intended for easier USB cable routing in the vehicle.

## USB Detection

On macOS, the board commonly appears as an Espressif USB Serial/JTAG device. Verify the exact port before flashing:

```bash
.venv/bin/pio device list
```

Do not flash unless the port is clearly the intended ESP32-S3 board.

## Touch

The firmware does not currently use touch input. Some T-Display-S3 variants include a touch controller, but this project routes controls through the two physical buttons.
