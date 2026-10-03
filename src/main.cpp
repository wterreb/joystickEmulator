#include <Arduino.h>

namespace {

constexpr int kElrsRxPin = 16;  // ESP32 receives on GPIO16 from ELRS TX
constexpr int kElrsTxPin = 17;  // ESP32 transmits on GPIO17 to ELRS RX
constexpr uint32_t kElrsBaud = 420000;
constexpr int kDataUartRxPin = 26;  // Optional RX pin for downstream UART input
constexpr int kDataUartTxPin = 25;  // TX pin used to stream joystick X,Y
constexpr uint32_t kDataUartBaud = 115200;
constexpr uint32_t kDataTxIntervalMs = 20;
constexpr uint32_t kDetectPrintIntervalMs = 1000;
constexpr uint32_t kStickHintThreshold = 120;
constexpr uint32_t kCrsfFailsafeTimeoutMs = 500;

// Default mapping for many EdgeTX/OpenTX profiles is AETR:
// CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw.
// Update these indexes later if your model mapping differs.
constexpr uint8_t kOutputChannelX = 0;  // CH1 by default
constexpr uint8_t kOutputChannelY = 1;  // CH2 by default

constexpr uint8_t kMaxFrameLen = 64;
constexpr uint8_t kCrsfAddressFlightController = 0xC8;
constexpr uint8_t kCrsfFrameTypeRcChannelsPacked = 0x16;
constexpr uint8_t kRcPayloadLen = 22;
constexpr uint32_t kPrintIntervalMs = 100;
constexpr uint16_t kCrsfMin = 172;
constexpr uint16_t kCrsfCenter = 992;
constexpr uint16_t kCrsfMax = 1811;
constexpr int16_t kNormalizeDeadband = 20;

HardwareSerial ElrsUart(2);
HardwareSerial DataUart(1);
uint16_t g_channels[16] = {0};
uint16_t g_prevChannels[16] = {0};
uint32_t g_channelActivity[16] = {0};
bool g_channelsValid = false;
uint32_t g_lastPrintMs = 0;
uint32_t g_lastDataTxMs = 0;
uint32_t g_lastDetectPrintMs = 0;
uint32_t g_lastCrsfFrameMs = 0;

const char* kChannelLabels[16] = {
  "Roll", "Pitch", "Throttle", "Yaw", "AUX1", "AUX2", "AUX3", "AUX4",
  "AUX5", "AUX6", "AUX7",     "AUX8", "AUX9", "AUX10", "AUX11", "AUX12"};

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
  Serial.print("RC: ");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(i + 1);
    Serial.print('(');
    Serial.print(kChannelLabels[i]);
    Serial.print(')');
    Serial.print('=');
    Serial.print(channels[i]);
    if (i != 15) {
      Serial.print("  ");
    }
  }
  Serial.println();
}

int16_t normalizeCrsfTo1000(uint16_t raw) {
  if (raw == kCrsfCenter) {
    return 0;
  }

  int32_t normalized = 0;

  if (raw > kCrsfCenter) {
    normalized = static_cast<int32_t>(raw - kCrsfCenter) * 1000 /
                 static_cast<int32_t>(kCrsfMax - kCrsfCenter);
  } else {
    normalized = -static_cast<int32_t>(kCrsfCenter - raw) * 1000 /
                 static_cast<int32_t>(kCrsfCenter - kCrsfMin);
  }

  if (normalized > 1000) {
    normalized = 1000;
  }
  if (normalized < -1000) {
    normalized = -1000;
  }

  return static_cast<int16_t>(normalized);
}

int16_t applyDeadband(int16_t value) {
  if (value > -kNormalizeDeadband && value < kNormalizeDeadband) {
    return 0;
  }
  return value;
}

void updateActivity(const uint16_t* channels) {
  for (uint8_t i = 0; i < 16; i++) {
    uint16_t current = channels[i];
    uint16_t previous = g_prevChannels[i];
    uint16_t delta = current > previous ? (current - previous) : (previous - current);
    g_channelActivity[i] += delta;
    g_prevChannels[i] = current;
  }
}

void printLikelyRightStickChannels() {
  uint8_t firstIndex = 0;
  uint8_t secondIndex = 1;

  for (uint8_t i = 0; i < 16; i++) {
    if (g_channelActivity[i] > g_channelActivity[firstIndex]) {
      secondIndex = firstIndex;
      firstIndex = i;
    } else if (i != firstIndex && g_channelActivity[i] > g_channelActivity[secondIndex]) {
      secondIndex = i;
    }
  }

  Serial.print("Hint: move only right stick. Most active channels now: CH");
  Serial.print(firstIndex + 1);
  Serial.print(" (");
  Serial.print(kChannelLabels[firstIndex]);
  Serial.print(") and CH");
  Serial.print(secondIndex + 1);
  Serial.print(" (");
  Serial.print(kChannelLabels[secondIndex]);
  Serial.println(")");

  if (g_channelActivity[firstIndex] < kStickHintThreshold ||
      g_channelActivity[secondIndex] < kStickHintThreshold) {
    Serial.println("Hint quality is low. Move only the right stick farther for a cleaner detect.");
  }

  for (uint8_t i = 0; i < 16; i++) {
    g_channelActivity[i] = 0;
  }
}

void sendJoystickUartFrame(int16_t x, int16_t y) {
  DataUart.print(x);
  DataUart.print(',');
  DataUart.print(y);
  DataUart.print("\r\n");
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
          updateActivity(g_channels);
          g_lastCrsfFrameMs = millis();
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
  Serial.println("Output UART1: 115200 on GPIO25(TX), GPIO26(RX)");
  Serial.println("Default output channels: CH1->X, CH2->Y (update after live detect)");
  Serial.println("========================================");

  ElrsUart.begin(kElrsBaud, SERIAL_8N1, kElrsRxPin, kElrsTxPin);
  DataUart.begin(kDataUartBaud, SERIAL_8N1, kDataUartRxPin, kDataUartTxPin);
  Serial.println("Output UART1 initialized at 115200");
}

void loop() {
  processCrsf();

  const uint32_t now = millis();

  if (g_channelsValid && (now - g_lastCrsfFrameMs) > kCrsfFailsafeTimeoutMs) {
    g_channelsValid = false;
    Serial.println("Warning: CRSF stream timeout");
  }

  if (g_channelsValid && (now - g_lastPrintMs) >= kPrintIntervalMs) {
    const int16_t normX = applyDeadband(normalizeCrsfTo1000(g_channels[kOutputChannelX]));
    const int16_t normY = applyDeadband(normalizeCrsfTo1000(g_channels[kOutputChannelY]));

    printChannels(g_channels);
    Serial.print("Norm: X(CH");
    Serial.print(kOutputChannelX + 1);
    Serial.print(")=");
    Serial.print(normX);
    Serial.print("  Y(CH");
    Serial.print(kOutputChannelY + 1);
    Serial.print(")=");
    Serial.print(normY);
    Serial.print("  deadband=");
    Serial.print(kNormalizeDeadband);
    Serial.println();
    g_lastPrintMs = now;
  }

  if (g_channelsValid && (now - g_lastDataTxMs) >= kDataTxIntervalMs) {
    sendJoystickUartFrame(applyDeadband(normalizeCrsfTo1000(g_channels[kOutputChannelX])),
                          applyDeadband(normalizeCrsfTo1000(g_channels[kOutputChannelY])));
    g_lastDataTxMs = now;
  }

  if (g_channelsValid && (now - g_lastDetectPrintMs) >= kDetectPrintIntervalMs) {
    printLikelyRightStickChannels();
    g_lastDetectPrintMs = now;
  }
}
