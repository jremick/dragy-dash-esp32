# Dragy BLE Protocol Notes

These are sanitized implementation notes used by this firmware. They are not raw packet captures.

## Discovery

Observed Dragy Pro devices advertise with:

- Local name shape: `DRG70-xxxxxx`
- Primary service UUID: `FD00`

The firmware treats a device as a Dragy candidate when:

- It advertises service `FD00`, or
- Its name starts with `DRG`, or
- Its name contains `Dragy`.

## Primary Service

Service `FD00` contains the characteristics used by this firmware:

- `FD02`: telemetry notification stream.
- `FD03`: handshake challenge/response.
- `FD04`: battery/status values.

## Handshake

To start telemetry:

1. Read `FD03`.
2. Treat the first two bytes as challenge bytes `a` and `b`.
3. Write four bytes to `FD03`: `[a, b, a XOR b, a AND b]`.
4. Subscribe to `FD02` notifications for live telemetry.

## Telemetry

`FD02` streams fragmented UBX NAV-PVT-like packets after handshake:

- UBX sync: `B5 62`
- Message class/id: `01 07`
- Payload length: 92 bytes
- Full frame length: 100 bytes including sync and checksum

The firmware reassembles fragments and validates the UBX checksum before decoding.

Decoded fields:

- Fix quality
- Satellite count
- Altitude
- Horizontal accuracy
- Ground speed, using NAV-PVT `gSpeed` in mm/s
- Heading

Speed is converted to km/h.

## Battery

`FD04` appears to expose battery percentage in byte 1 when the value is between 0 and 100.

## Filtering

No-fix samples are ignored for:

- Distance
- Peak speed
- Rolling averages

They can still be reflected in the connection/fix state shown on the display.
