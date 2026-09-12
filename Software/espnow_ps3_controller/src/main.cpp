#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Ps3Controller.h>

// Replace these values before uploading.
constexpr uint8_t kRobotMac[] = {0x9C, 0xCC, 0x01, 0xE3, 0x7D, 0xC4}; //9C:CC:01:E3:7D:C4
constexpr uint8_t kEspNowChannel = 1;
constexpr uint32_t kSendIntervalMs = 10;
constexpr uint32_t kEspNowControllerMagic = 0x42524F54;
constexpr uint8_t kEspNowControllerVersion = 1;

// These values must remain identical to the robot receiver.
enum EspNowControllerButton : uint16_t {
  ESP_NOW_BUTTON_DOWN = 1 << 0,
  ESP_NOW_BUTTON_LEFT = 1 << 1,
  ESP_NOW_BUTTON_UP = 1 << 2,
  ESP_NOW_BUTTON_RIGHT = 1 << 3,
  ESP_NOW_BUTTON_CIRCLE = 1 << 4,
  ESP_NOW_BUTTON_CROSS = 1 << 5,
  ESP_NOW_BUTTON_SQUARE = 1 << 6,
  ESP_NOW_BUTTON_R1 = 1 << 7,
  ESP_NOW_BUTTON_R2 = 1 << 8,
};

struct EspNowControllerPacket {
  uint32_t magic;
  uint8_t version;
  bool connected;
  int8_t leftStickY;
  int8_t rightStickX;
  uint16_t buttons;
} __attribute__((packed));

static_assert(sizeof(EspNowControllerPacket) == 10, "ESP-NOW packet layout changed");

uint16_t readButtons() {
  uint16_t buttons = 0;
  if (Ps3.data.button.down) buttons |= ESP_NOW_BUTTON_DOWN;
  if (Ps3.data.button.left) buttons |= ESP_NOW_BUTTON_LEFT;
  if (Ps3.data.button.up) buttons |= ESP_NOW_BUTTON_UP;
  if (Ps3.data.button.right) buttons |= ESP_NOW_BUTTON_RIGHT;
  if (Ps3.data.button.circle) buttons |= ESP_NOW_BUTTON_CIRCLE;
  if (Ps3.data.button.cross) buttons |= ESP_NOW_BUTTON_CROSS;
  if (Ps3.data.button.square) buttons |= ESP_NOW_BUTTON_SQUARE;
  if (Ps3.data.button.r1) buttons |= ESP_NOW_BUTTON_R1;
  if (Ps3.data.button.r2) buttons |= ESP_NOW_BUTTON_R2;
  return buttons;
}

void sendControllerPacket() {
  EspNowControllerPacket packet = {};
  packet.magic = kEspNowControllerMagic;
  packet.version = kEspNowControllerVersion;
  packet.connected = Ps3.isConnected();
  if (packet.connected) {
    packet.leftStickY = static_cast<int8_t>(Ps3.data.analog.stick.ly);
    packet.rightStickX = static_cast<int8_t>(Ps3.data.analog.stick.rx);
    packet.buttons = static_cast<uint16_t>(readButtons());
  }

  esp_err_t result = esp_now_send(kRobotMac, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW send failed: %s\n", esp_err_to_name(result));
  }
}

void onPs3Connect() {
  Serial.println("PS3 controller connected");
}

void onPs3Disconnect() {
  Serial.println("PS3 controller disconnected");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
  Serial.print("Sender Wi-Fi MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("ESP-NOW channel: %u\n", kEspNowChannel);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, kRobotMac, sizeof(kRobotMac));
  peerInfo.channel = kEspNowChannel;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add robot as ESP-NOW peer");
    return;
  }

  Ps3.attachOnConnect(onPs3Connect);
  Ps3.attachOnDisconnect(onPs3Disconnect);
  Ps3.begin();
  Serial.print("PS3 host Bluetooth MAC: ");
  Serial.println(Ps3.getAddress());
  Serial.println("Pair the PS3 controller with the printed Bluetooth MAC.");
}

void loop() {
  static uint32_t lastSendMs = 0;
  uint32_t nowMs = millis();

  if (nowMs - lastSendMs >= kSendIntervalMs) {
    lastSendMs = nowMs;
    sendControllerPacket();
  }
}
