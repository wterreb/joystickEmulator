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

## Joystick Output UART Wiring

The firmware now outputs normalized joystick X,Y values over a dedicated UART
instead of CAN.

Use these ESP32 pins for the output UART:

- ESP32 GPIO25 (UART1 TX) -> Receiver UART RX
- ESP32 GPIO26 (UART1 RX) -> Optional, only needed if you later want inbound UART data
- ESP32 GND -> Receiver GND

Important notes:

- Output UART baudrate is 115200, 8N1, no CTS/RTS flow control.
- USB-C debug output remains on the normal USB serial at 115200.

Current firmware behavior:

- It sends one UART line every 20 ms (50 Hz) on UART1 TX (GPIO25).
- By default it sends CH1 as X and CH2 as Y until you confirm the exact right-stick channels.
- X and Y are normalized to -1000..+1000 (center stick = 0).
- A center deadband is applied (default +/-20) so small jitter reports as 0.
- UART payload format is text: `X,Y\r\n` (example: `-123,456\r\n`).
- USB serial output prints all channels and a live hint of the two most-active channels
	while you move the right stick.

Deadband tuning:

- Adjust `kNormalizeDeadband` in `src/main.cpp` to change center sensitivity.
- Lower value = more sensitive around center, higher value = more stable zero.

## Notes

The MCU board used is ESP32 WeAct ESP32 Development Board TYPE-C CH340K WiFi+Bluetooth Dual Core ESP32-DOWD-V3.
The CRSF receiver is HelloRadio HR8E ELRS 2.4G 9-Channel PWM Power Supply DC 4.5-7.4 V Receiver Dual Antenna

Reconnect the receiver to the following pings: receiver TX to pin 16, receiver RX to pin 17, GND to GND. Power it from 5 V, with a shared GND.
