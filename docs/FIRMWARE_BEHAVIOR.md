# Firmware Behavior

## Display Pages

The screen is metric-only and refreshed every 100 ms / 10 Hz. Redraws are region-based to reduce flicker.

- Page 1: large live speed, 5 km average, satellites, horizontal accuracy.
- Page 2: large live speed, 1 km / 5 km / 10 km rolling averages.
- Page 3: large live speed, peak speed, distance, altitude.

The display is flipped 180 degrees with `tft.setRotation(3)` to make cable routing easier in the vehicle.

## Controls

- Quick left button tap: previous page.
- Quick right button tap: next page.
- Long left button: reset run stats in RAM.
- Long right button: disconnect and pause BLE scanning; long right again resumes scanning.
- Hold both buttons: cycle brightness through dim, medium, and high.

Stats reset clears rolling averages, peak speed, and distance. It does not power-cycle the ESP32 or write anything to persistent storage.

## Runtime States

The main speed panel can show status text before live speed is usable:

- `SCAN`: searching for a Dragy advertisement.
- `WAIT GPS`: connected, but telemetry is not usable yet.
- `BUSY?`: connected, but Dragy did not start streaming within the timeout.
- `STREAM`: packet stream stopped after it had started.
- `PAUSED`: BLE scanning was paused by long-press control.

The status row shows:

- `WAIT`
- `WAIT GPS`
- `READY`
- `ROLLING`

## GPS Quality

The status row includes a GPS quality dot:

- Green: 3D fix, at least 8 satellites, horizontal accuracy at or below 2.5 m.
- Yellow: usable 3D/estimated fix, at least 6 satellites, horizontal accuracy at or below 5 m.
- Orange/red: weak or unusable GPS.

## Sampling And Stats

- Dragy telemetry is notification-driven from BLE characteristic `FD02`.
- The firmware does not poll speed packets.
- No-fix samples do not feed distance, peak speed, or rolling averages.
- Rolling averages keep a distance window long enough for the last 15 km.
- The ESP32 stores compact distance/time segments instead of raw CSV history.

## Deliberately Not Included

- Imperial units.
- 0-60 or 0-100 timing.
- Lateral or longitudinal G.
- Wi-Fi, SD card, session saving, or settings screens.
