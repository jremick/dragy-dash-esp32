# Firmware Behavior

## Display Pages

The screen refreshes at 10 Hz (every 100 ms) using region-cached redraws to minimize flicker. Unit formatting is selected at build time (`lilygo_t_display_s3` for metric, `lilygo_t_display_s3_imperial` for imperial).

- **Page 1:** large live speed, `5 km` (`3 mi`) rolling average, satellites, horizontal accuracy (`m` / `ft`).
- **Page 2:** large live speed, `1 / 5 / 10 km` (`1 / 3 / 5 mi`) rolling averages.
- **Page 3:** large live speed, peak speed, distance (`km` / `mi`), altitude (`m` / `ft`).

The display is flipped 180 degrees with `tft.setRotation(3)` to make USB-C cable routing easier in the vehicle.

## Controls

- **Quick left button tap:** previous page (persisted to NVS).
- **Quick right button tap:** next page (persisted to NVS).
- **Long left button hold:** reset run stats in RAM.
- **Long right button hold:** disconnect and pause BLE scanning; long right again resumes scanning.
- **Hold both buttons:** cycle brightness through `DIM`, `MID`, and `HIGH` (persisted to NVS).

Stats reset clears rolling averages, peak speed, and distance. Only active page and brightness are written to NVS (solely on explicit button actions).

## Runtime States

The main speed panel shows status text before live speed is usable:

- `SCAN`: asynchronously searching for the strongest-RSSI Dragy advertisement.
- `CONNECT`: connecting to the discovered Dragy peripheral.
- `HANDSHAKE`: subscribing to `FD02`/`FD04` and performing the `FD03` handshake.
- `WAIT GPS`: connected, waiting for a usable GNSS fix.
- `BUSY?`: connected, but Dragy did not start streaming within 8 seconds (often because another app is connected).
- `STREAM`: packet stream stalled for more than 4 seconds after starting.
- `PAUSED`: BLE scanning paused by long-press control.

The status row shows `WAIT`, `WAIT GPS`, `READY`, or `ROLLING`, alongside Dragy battery percentage and fix type when available.

## GPS Quality

The status row includes a GPS quality dot:

- **Green:** 3D fix, at least 8 satellites, horizontal accuracy <= 2.5 m.
- **Yellow:** 3D or estimated fix, at least 6 satellites, horizontal accuracy <= 5.0 m.
- **Orange / Red:** weak or unusable GPS.

## Sampling, Threading, And Stats

- NimBLE 2.x callbacks push raw `FD02` notification chunks into a FreeRTOS queue and update atomic battery/disconnect flags; all parsing, stats tracking, Serial logging, and display updates run exclusively on the Arduino `loop()` task.
- Stationary GNSS jitter is filtered with 5 km/h start and 2 km/h stop hysteresis before feeding display, peak speed, distance, and rolling averages.
- Unusable samples (no-fix or zero satellites) are excluded from distance, peak speed, and rolling averages.
- Rolling averages use a zero-heap fixed circular ring buffer of 1,600 x 10 m distance buckets (~12.8 KB static RAM, O(1) per packet) and prefer UBX `iTOW` timestamps over arrival jitter when advancing cleanly.

## Deliberately Not Included

- 0-60 or 0-100 timing.
- Lateral or longitudinal G.
- Wi-Fi, SD card, session CSV logging, or on-device settings menus.
