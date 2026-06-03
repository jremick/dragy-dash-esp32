# Contributing

Contributions are welcome, but keep the project privacy-safe and hardware-safe.

## Ground Rules

- Do not commit raw BLE captures, route traces, full Dragy identifiers, unique serial logs, or personal device details.
- Keep protocol examples sanitized and reproducible.
- Keep the firmware metric-only unless there is a clear product reason to add complexity.
- Do not add settings screens, Wi-Fi, SD logging, or acceleration timing without discussing the scope first.
- This project is unofficial and experimental; wording should not imply Dragy or LilyGO endorsement.

## Local Checks

Project-local PlatformIO:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade platformio
.venv/bin/pio run
```

Global PlatformIO:

```bash
pio run
```

## Hardware Testing

Flash only to a confirmed LilyGO T-Display-S3 or compatible ESP32-S3 board. Before posting logs or screenshots, redact personal identifiers and route data.
