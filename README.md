# joystickEmulator

Minimal PlatformIO project for the WeAct ESP32 Development Board TYPE-C CH340K WiFi+Bluetooth Dual Core ESP32-DOWD-V3.

This project is configured for the ESP32 Arduino framework and prints a simple startup message over the serial port.

## Requirements

- Python 3
- PlatformIO
- USB cable for the ESP32 board

## Install PlatformIO

```bash
python -m pip install --user platformio
```

## Build

```bash
pio run
```

## Upload

```bash
pio run --target upload
```

## Monitor serial output

```bash
pio device monitor
```

## Notes

The board you listed is an ESP32 board, not an STM32 board. This project uses the ESP32 toolchain and Arduino core instead of STM32Cube.
