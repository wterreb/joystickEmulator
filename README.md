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

If `pio` is not recognized on Windows, use:

```bash
python -m platformio run
```

## Upload

```bash
pio run --target upload
```

Fallback:

```bash
python -m platformio run --target upload
```

## Monitor serial output

```bash
pio device monitor
```

Fallback:

```bash
python -m platformio device monitor
```

## ELRS Receiver Wiring (CRSF)

Use UART2 on the ESP32:

- ELRS TX -> ESP32 GPIO16 (UART2 RX)
- ELRS RX -> ESP32 GPIO17 (UART2 TX)
- ELRS GND -> ESP32 GND
- ELRS VCC -> 5V or 3.3V only if your specific receiver supports it (check receiver datasheet)

Notes:

- CRSF serial speed is 420000 baud and is configured in firmware.
- Channel values are printed on the USB serial console at 115200 baud.
- The firmware decodes and prints all 16 CRSF RC channels.

## Notes

The board you listed is an ESP32 board, not an STM32 board. This project uses the ESP32 toolchain and Arduino core instead of STM32Cube.
