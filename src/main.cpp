#include <Arduino.h>
#include <driver/twai.h>

namespace {

constexpr int kElrsRxPin = 16;  // ESP32 receives on GPIO16 from ELRS TX
constexpr int kElrsTxPin = 17;  // ESP32 transmits on GPIO17 to ELRS RX
constexpr uint32_t kElrsBaud = 420000;
constexpr int kCanRxPin = 22;   // ESP32 receives from CAN transceiver TXD
constexpr int kCanTxPin = 21;   // ESP32 transmits to CAN transceiver RXD
constexpr uint32_t kCanTxIntervalMs = 20;
constexpr uint32_t kDetectPrintIntervalMs = 1000;
constexpr uint32_t kStickHintThreshold = 120;
constexpr uint32_t kCrsfFailsafeTimeoutMs = 500;

// Default mapping for many EdgeTX/OpenTX profiles is AETR:
// CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw.
// Update these indexes later if your model mapping differs.
constexpr uint8_t kCanChannelX = 0;  // CH1 by default
constexpr uint8_t kCanChannelY = 1;  // CH2 by default

constexpr uint16_t kCanFrameId = 0x120;
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
uint16_t g_channels[16] = {0};
uint16_t g_prevChannels[16] = {0};
uint32_t g_channelActivity[16] = {0};
bool g_channelsValid = false;
bool g_canReady = false;
uint32_t g_lastPrintMs = 0;
uint32_t g_lastCanTxMs = 0;
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

bool initCan() {
  twai_general_config_t generalConfig =
      TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(kCanTxPin),
                                  static_cast<gpio_num_t>(kCanRxPin), TWAI_MODE_NORMAL);
  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&generalConfig, &timingConfig, &filterConfig);
  if (err != ESP_OK) {
    Serial.print("CAN driver install failed, err=");
    Serial.println(err);
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.print("CAN start failed, err=");
    Serial.println(err);
    twai_driver_uninstall();
    return false;
  }

  return true;
}

void sendCanFrame(int16_t chX, int16_t chY) {
  if (!g_canReady) {
    return;
  }

  twai_message_t msg = {};
  msg.identifier = kCanFrameId;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;

  msg.data[0] = static_cast<uint8_t>(chX & 0xFF);
  msg.data[1] = static_cast<uint8_t>((chX >> 8) & 0xFF);
  msg.data[2] = static_cast<uint8_t>(chY & 0xFF);
  msg.data[3] = static_cast<uint8_t>((chY >> 8) & 0xFF);

  // Echo source channels in payload to simplify downstream bring-up.
  msg.data[4] = static_cast<uint8_t>(kCanChannelX + 1);
  msg.data[5] = static_cast<uint8_t>(kCanChannelY + 1);
  msg.data[6] = 0;
  msg.data[7] = 0;

  twai_transmit(&msg, 0);
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
  Serial.println("CAN (TWAI): 1Mbps on GPIO21(TX), GPIO22(RX)");
  Serial.println("Default CAN channels: CH1->X, CH2->Y (update after live detect)");
  Serial.println("========================================");

  ElrsUart.begin(kElrsBaud, SERIAL_8N1, kElrsRxPin, kElrsTxPin);
  g_canReady = initCan();

  if (g_canReady) {
    Serial.println("CAN initialized at 1Mbps");
  } else {
    Serial.println("CAN initialization failed");
  }
}

void loop() {
  processCrsf();

  const uint32_t now = millis();

  if (g_channelsValid && (now - g_lastCrsfFrameMs) > kCrsfFailsafeTimeoutMs) {
    g_channelsValid = false;
    Serial.println("Warning: CRSF stream timeout");
  }

  if (g_channelsValid && (now - g_lastPrintMs) >= kPrintIntervalMs) {
    const int16_t normX = applyDeadband(normalizeCrsfTo1000(g_channels[kCanChannelX]));
    const int16_t normY = applyDeadband(normalizeCrsfTo1000(g_channels[kCanChannelY]));

    printChannels(g_channels);
    Serial.print("Norm: X(CH");
    Serial.print(kCanChannelX + 1);
    Serial.print(")=");
    Serial.print(normX);
    Serial.print("  Y(CH");
    Serial.print(kCanChannelY + 1);
    Serial.print(")=");
    Serial.print(normY);
    Serial.print("  deadband=");
    Serial.print(kNormalizeDeadband);
    Serial.println();
    g_lastPrintMs = now;
  }

  if (g_channelsValid && (now - g_lastCanTxMs) >= kCanTxIntervalMs) {
    sendCanFrame(applyDeadband(normalizeCrsfTo1000(g_channels[kCanChannelX])),
                 applyDeadband(normalizeCrsfTo1000(g_channels[kCanChannelY])));
    g_lastCanTxMs = now;
  }

  if (g_channelsValid && (now - g_lastDetectPrintMs) >= kDetectPrintIntervalMs) {
    printLikelyRightStickChannels();
    g_lastDetectPrintMs = now;
  }
}
