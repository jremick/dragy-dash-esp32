# Contributing

Contributions are welcome, but keep the project privacy-safe, hardware-safe, and focused.

## Ground Rules

- Do not commit raw BLE captures, route traces, full Dragy identifiers, unique serial logs, or personal device details.
- Keep protocol examples sanitized and reproducible.
- Keep pure decoding, filtering, and statistics logic inside `lib/dragy_core/` with native unit test coverage in `test/test_dragy_core/`.
- Keep the on-device UI menu-free; unit choices belong in compile-time PlatformIO environments rather than settings screens.
- Do not add Wi-Fi, SD logging, or acceleration timing without discussing scope first.
- This project is unofficial and experimental; wording should not imply Dragy or LilyGO endorsement.

## Local Checks

Project-local PlatformIO:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade platformio
.venv/bin/pio test -e native
.venv/bin/pio run -e lilygo_t_display_s3
.venv/bin/pio run -e lilygo_t_display_s3_imperial
```

Global PlatformIO:

```bash
pio test -e native
pio run -e lilygo_t_display_s3
pio run -e lilygo_t_display_s3_imperial
```

## Hardware Testing

Flash only to a confirmed LilyGO T-Display-S3 or compatible ESP32-S3 board. Before posting logs or screenshots, confirm personal identifiers and route data are redacted.
