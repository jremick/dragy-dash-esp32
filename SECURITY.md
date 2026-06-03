# Security Policy

## Reporting

Please do not open public issues containing raw BLE captures, full Dragy identifiers, route traces, unique serial logs, or personal device identifiers.

For public discussion, redact:

- Dragy device suffixes and persistent BLE identifiers
- route or location data
- serial ports or logs that uniquely identify a device
- packet captures that may identify a specific device or route

If you believe the firmware exposes sensitive data by default, open a minimal issue with redacted reproduction steps and note that private details are available if needed.

## Supported Versions

This is an experimental project. Only the current `main` branch is actively maintained.

## Scope

In scope:

- accidental disclosure of private telemetry or device identifiers
- unsafe diagnostic defaults
- dependency or build-chain issues that affect this repository

Out of scope:

- vulnerabilities in Dragy or LilyGO hardware, firmware, or official apps
- bypassing access controls on devices you do not own
- unsafe driving behavior or operation outside local laws
