# Dragy BLE Protocol Notes

These are sanitized implementation notes used by this firmware (shared with the companion [DragyDash iOS app](https://github.com/jremick/dragy-dash)). They are not raw packet captures.

## Discovery

Observed Dragy Pro devices advertise with:

- Local name shape: `DRG70-xxxxxx`
- Primary service UUID: `FD00`

The firmware treats a device as a Dragy candidate when (case-insensitive):

- It advertises service `FD00`, or
- Its name starts with `DRG`, or
- Its name contains `Dragy`.

When multiple candidates appear during a scan window, the firmware selects the candidate with the strongest RSSI.

## Primary Service

Service `FD00` contains the characteristics used by this firmware:

- `FD02`: telemetry notification stream.
- `FD03`: handshake challenge/response.
- `FD04`: battery/status values.

## Handshake

To start telemetry:

1. Subscribe to `FD02` notifications (and optional `FD04` status).
2. Read `FD03`.
3. Treat the first two bytes as challenge bytes `a` and `b`.
4. Write four bytes to `FD03`: `[a, b, a XOR b, a AND b]`.
5. Receive fragmented UBX telemetry on `FD02`.

## Telemetry

`FD02` streams fragmented UBX `NAV-PVT`-like packets after handshake:

- UBX sync: `B5 62`
- Message class/id: `01 07`
- Payload length: 92 bytes
- Full frame length: 100 bytes including sync and checksum

`UbxReassembler` validates the UBX Fletcher-8 checksum before advancing past candidate sync markers so false `B5 62` matches do not swallow valid subsequent frames.

Decoded fields:

- `iTOW` GPS millisecond time-of-week (used for sample-to-sample elapsed deltas when advancing cleanly)
- Fix quality (`fixType`)
- Satellite count (`numSV`)
- Altitude (`hMSL`)
- Horizontal accuracy (`hAcc`)
- Ground speed (`gSpeed` in mm/s -> km/h, optionally converted to mph in Imperial builds)
- Heading (`headMot`)

## Battery

`FD04` exposes battery percentage in byte 1 when the value is between 0 and 100.

## Filtering

Unusable samples (no-fix or zero satellites) and stationary GNSS jitter (below 5 km/h start / 2 km/h stop hysteresis) are excluded from:

- Distance
- Peak speed
- Rolling averages
