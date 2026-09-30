#include <Arduino.h>

namespace {

constexpr int kElrsRxPin = 16;  // ESP32 receives on GPIO16 from ELRS TX
constexpr int kElrsTxPin = 17;  // ESP32 transmits on GPIO17 to ELRS RX
constexpr uint32_t kElrsBaud = 420000;
constexpr uint8_t kMaxFrameLen = 64;
constexpr uint8_t kCrsfAddressFlightController = 0xC8;
constexpr uint8_t kCrsfFrameTypeRcChannelsPacked = 0x16;
constexpr uint8_t kRcPayloadLen = 22;
constexpr uint32_t kPrintIntervalMs = 100;

HardwareSerial ElrsUart(2);
uint16_t g_channels[16] = {0};
bool g_channelsValid = false;
uint32_t g_lastPrintMs = 0;

uint8_t crsfCrc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;

  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void decodePackedRcChannels(const uint8_t* payload, uint16_t* outChannels) {
  uint32_t bitBuffer = 0;
  uint8_t bitsInBuffer = 0;
  uint8_t payloadIndex = 0;

  for (uint8_t ch = 0; ch < 16; ch++) {
    while (bitsInBuffer < 11) {
      bitBuffer |= static_cast<uint32_t>(payload[payloadIndex++]) << bitsInBuffer;
      bitsInBuffer += 8;
    }

    outChannels[ch] = static_cast<uint16_t>(bitBuffer & 0x07FF);
    bitBuffer >>= 11;
    bitsInBuffer -= 11;
  }
}

void printChannels(const uint16_t* channels) {
  Serial.print("CH: ");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(i + 1);
    Serial.print('=');
    Serial.print(channels[i]);
    if (i != 15) {
      Serial.print("  ");
    }
  }
  Serial.println();
}

void processCrsf() {
  static enum { kWaitAddress, kWaitLength, kReadBody } state = kWaitAddress;
  static uint8_t address = 0;
  static uint8_t length = 0;
  static uint8_t body[kMaxFrameLen] = {0};
  static uint8_t bodyIndex = 0;

  while (ElrsUart.available() > 0) {
    const uint8_t byteIn = static_cast<uint8_t>(ElrsUart.read());

    switch (state) {
      case kWaitAddress:
        if (byteIn == kCrsfAddressFlightController) {
          address = byteIn;
          state = kWaitLength;
        }
        break;

      case kWaitLength:
        if (byteIn < 2 || byteIn > kMaxFrameLen) {
          state = kWaitAddress;
          break;
        }
        length = byteIn;
        bodyIndex = 0;
        state = kReadBody;
        break;

      case kReadBody:
        body[bodyIndex++] = byteIn;
        if (bodyIndex < length) {
          break;
        }

        state = kWaitAddress;

        if (length < 2) {
          break;
        }

        const uint8_t receivedCrc = body[length - 1];
        const uint8_t calculatedCrc = crsfCrc8(body, length - 1);
        if (receivedCrc != calculatedCrc) {
          break;
        }

        const uint8_t frameType = body[0];
        const uint8_t* payload = &body[1];
        const uint8_t payloadLen = static_cast<uint8_t>(length - 2);

        if (address == kCrsfAddressFlightController &&
            frameType == kCrsfFrameTypeRcChannelsPacked &&
            payloadLen == kRcPayloadLen) {
          decodePackedRcChannels(payload, g_channels);
          g_channelsValid = true;
        }
        break;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("========================================");
  Serial.println("ESP32 ELRS (CRSF) UART Channel Monitor");
  Serial.println("Board: ESP32-DOWD-V3");
  Serial.println("USB Serial: 115200");
  Serial.println("ELRS UART2: 420000 on GPIO16(RX), GPIO17(TX)");
  Serial.println("Wiring: ELRS TX -> GPIO16, ELRS RX -> GPIO17, GND -> GND");
  Serial.println("========================================");

  ElrsUart.begin(kElrsBaud, SERIAL_8N1, kElrsRxPin, kElrsTxPin);
}

void loop() {
  processCrsf();

  const uint32_t now = millis();
  if (g_channelsValid && (now - g_lastPrintMs) >= kPrintIntervalMs) {
    printChannels(g_channels);
    g_lastPrintMs = now;
  }
}
