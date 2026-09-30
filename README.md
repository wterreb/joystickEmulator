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

## CAN Driver Module Wiring (1 Mbps)

The ESP32 has the CAN controller built in, but needs an external CAN transceiver
module (for example SN65HVD230 or TJA1050 style board).

Use these ESP32 pins for CAN:

- ESP32 GPIO21 (CAN TX) -> Transceiver RXD
- ESP32 GPIO22 (CAN RX) -> Transceiver TXD
- ESP32 GND -> Transceiver GND
- ESP32 3.3V or 5V -> Transceiver VCC (match your transceiver board requirement)
- Transceiver CANH -> CAN bus CANH
- Transceiver CANL -> CAN bus CANL

Important notes:

- Firmware config uses 1 Mbps CAN bitrate.
- Use 120 ohm termination at each end of the CAN bus.
- Keep grounds shared between all CAN nodes.
- ESP32 GPIO is 3.3V logic, so make sure your transceiver module is logic-compatible.

Current firmware behavior:

- It sends a CAN frame every 20 ms (50 Hz).
- Frame ID is 0x120.
- By default it sends CH1 as X and CH2 as Y until you confirm the exact right-stick channels.
- X and Y are normalized to -1000..+1000 (center stick = 0).
- A center deadband is applied (default +/-20) so small jitter reports as 0.
- CAN payload packs X and Y as little-endian signed int16 values in bytes [0..3].
- USB serial output prints all channels and a live hint of the two most-active channels
	while you move the right stick.

Deadband tuning:

- Adjust `kNormalizeDeadband` in `src/main.cpp` to change center sensitivity.
- Lower value = more sensitive around center, higher value = more stable zero.

## Notes

The MCU board used is ESP32 WeAct ESP32 Development Board TYPE-C CH340K WiFi+Bluetooth Dual Core ESP32-DOWD-V3.
The CRSF receiver is HelloRadio HR8E ELRS 2.4G 9-Channel PWM Power Supply DC 4.5-7.4 V Receiver Dual Antenna
