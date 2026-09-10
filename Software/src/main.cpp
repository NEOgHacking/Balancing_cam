/*
A high speed balancing robot, running on an ESP32.

Wouter Klop
wouter@elexperiment.nl
For updates, see elexperiment.nl

Use at your own risk. This code is far from stable.

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
This basically means: if you use my code, acknowledge it.
Also, you have to publish all modifications.
*/

#define esp32_s3

#include <Arduino.h>
#include <Wire.h>
#include <FlySkyIBus.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Streaming.h>
#include <MPU6050.h>
#include <PID.h>
#include <AsyncTCP.h>
#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <fastStepper.h>
#include <Preferences.h>
#ifdef esp32_s3
#include <esp_now.h>
#include <TMC2209.h>
#include <LSM6DS3TRC.h>
#endif
#if defined(INPUT_PS3) && !defined(esp32_s3)
#include <Ps3Controller.h>
#endif
#include "driver/adc.h"
#include "esp_adc_cal.h"

// #define NEWPCB
// #define INPUT_PPM
// #define INPUT_IBUS
// #define INPUT_PS3

// Pin definitions
#ifdef esp32_s3
constexpr uint8_t kPinMotorEnable = 27;
constexpr uint8_t kPinLed = 34;
constexpr uint8_t kPinLedLeft = 4;
constexpr uint8_t kPinLedRight = 5;
constexpr uint8_t kPinMotorCurrent = 39;
constexpr uint8_t kPinFan = 21;
constexpr adc1_channel_t kBatteryAdcChannel = ADC1_CHANNEL_6;
constexpr long kTmcSerialBaudRate = 115200;
constexpr long kTmcSerialBaudRateStep = 115200;
constexpr int kTmcRunCurrent = 10;
constexpr int kTmcRxPin = 18;
constexpr int kTmcTxPin = 17;
constexpr int kTmcEnablePinLeft = 9;
constexpr int kTmcEnablePinRight = 12;
constexpr int kTmcStepPinLeft = 11;
constexpr int kTmcStepPinRight = 14;
constexpr int kTmcDirPinLeft = 10;
constexpr int kTmcDirPinRight = 13;
constexpr uint8_t kImuSdaPin = 37;
constexpr uint8_t kImuSclPin = 38;
#else
constexpr uint8_t kPinMotorEnable = 27;
constexpr uint8_t kPinMicroStep1 = 14;
constexpr uint8_t kPinMicroStep2 = 12;
constexpr uint8_t kPinMicroStep3 = 13;
constexpr uint8_t kPinLed = 32;
constexpr uint8_t kPinLedLeft = 33;
constexpr uint8_t kPinLedRight = 26;
constexpr uint8_t kPinMotorCurrent = 25;
constexpr adc1_channel_t kBatteryAdcChannel = ADC1_CHANNEL_6;
#endif

// Webserver constants
constexpr char kHttpUsername[] = "admin";
constexpr char kHttpPassword[] = "admin";
constexpr char kDefaultRobotName[] = "balancingrobot";
constexpr char kWifiApPassword[] = "turboturbo";
constexpr bool kFormatSPIFFSIfFailed = true;
constexpr size_t kRobotNameBufferSize = 63;
constexpr size_t kBtAddressBufferSize = 20;
constexpr uint32_t kPreferenceVersion = 1;
constexpr uint32_t kControlPeriodUs = 5000;
constexpr float kAngleErrorIntegralThreshold = 30.0f;
constexpr float kAngleErrorIntegralThresholdDuringSelfRight = kAngleErrorIntegralThreshold * 3.0f;
constexpr float kAngleEnableThreshold = 5.0f;
constexpr float kAngleDisableThreshold = 70.0f;
constexpr float kBatteryVoltageFilter = 0.99f;
constexpr float kBatteryVoltageScale = (100.0f + 3.3f) / 3.3f;
constexpr float kGyroSensitivity = 65.5f;
constexpr char kMqttBroker[] = "172.16.56.100";
constexpr uint16_t kMqttPort = 1883;
constexpr uint32_t kMqttPublishIntervalMs = 1000;
constexpr char kMqttAllCommandTopic[] = "robots/all/command";
constexpr size_t kMqttTopicBufferSize = 128;

enum ControlMode : uint8_t {
  CONTROL_ANGLE_ONLY = 0,
  CONTROL_ANGLE_POSITION = 1,
  CONTROL_ANGLE_SPEED = 2,
};

struct PlotSettings {
  bool enable = false;
  uint8_t prescaler = 4;
};

typedef union {
  uint8_t arr[6];
  struct {
    uint8_t grp;
    uint8_t cmd;
    union {
      float val;
      uint8_t valU8[4];
    } __attribute__((packed));
  } __attribute__((packed));
} cmd;

typedef union {
  struct {
    float val;
    uint8_t cmd;
    uint8_t checksum;
  } __attribute__((packed));
  uint8_t array[6];
} command;

struct RemoteControl {
  float speed = 0;
  float steer = 0;
  float speedGain = 0.15f;
  float steerGain = 0.25f;
  float speedOffset = 0.0f;
  bool selfRight = false;
  bool disableControl = false;
  bool overrideControl = false;
};

struct ControlState {
  bool active = false;
  bool overrideActive = false;
  bool selfRightActive = false;
  bool disableControlActive = false;
  float filteredSpeed = 0;
  float filteredSteer = 0;
  float averageMotorSpeed = 0;
  float angleErrorIntegral = 0;
  float batteryVoltage = 0;
  uint32_t lastInputMs = 0;
  uint32_t lastBatteryReportMs = 0;
  uint8_t plotTicker = 0;
};

// Global state
PlotSettings plot;
RemoteControl remoteControl;
ControlState controlState;

AsyncWebServer httpServer(80);
WebSocketsServer wsServer(81);
Preferences preferences;
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
char mqttStatusTopic[kMqttTopicBufferSize];
char mqttCommandTopic[kMqttTopicBufferSize];
uint32_t lastMqttPublishMs = 0;

void motLeftTimerFunction();
void motRightTimerFunction();
#if defined(INPUT_PS3) && !defined(esp32_s3)
void onPs3Notify();
void onPs3Connect();
void onPs3Disconnect();
#endif

#ifdef esp32_s3
constexpr uint32_t kEspNowControllerMagic = 0x42524F54;
constexpr uint8_t kEspNowControllerVersion = 1;
constexpr uint32_t kEspNowControllerTimeoutMs = 500;

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

volatile EspNowControllerPacket espNowControllerPacket = {};
volatile uint32_t espNowLastPacketMs = 0;
volatile bool espNowPacketReceived = false;

void onEspNowDataReceived(const uint8_t* macAddress, const uint8_t* data, int length) {
  if (length != sizeof(EspNowControllerPacket)) {
    return;
  }

  EspNowControllerPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != kEspNowControllerMagic || packet.version != kEspNowControllerVersion) {
    return;
  }

  memcpy((void*)&espNowControllerPacket, &packet, sizeof(packet));
  espNowLastPacketMs = millis();
  espNowPacketReceived = true;
}
#endif

uint8_t microStep = 16;
uint8_t motorCurrent = 150;
float maxStepSpeed = 1500.0f;

// Preference-backed defaults for PID angle controller (can be overridden from EEPROM)
float pref_pid1_p = 0.65f;
float pref_pid1_i = 1.0f;
float pref_pid1_d = 0.075f;
float pref_pid1_m = 15.0f;

PID pidAngle(cPID, kControlPeriodUs / 1000000.0f, 12.0f, -12.0f);
PID pidPos(cPD, kControlPeriodUs / 1000000.0f, 35.0f, -35.0f);
PID pidSpeed(cP, kControlPeriodUs / 1000000.0f, 35.0f, -35.0f);
ControlMode controlMode = CONTROL_ANGLE_POSITION;

#ifdef esp32_s3
LSM6DS3 imu(I2C_MODE, 0x6B);
bool imuReady = false;
#else
MPU6050 imu;
#endif
int16_t gyroOffset[3] = {0, 0, 0};
float accAngle = 0;
float filterAngle = 0;
float angleOffset = 2.0f;
float gyroFilterConstant = 0.996f;
float gyroGain = 1.0f;

bool noiseSourceEnable = false;
float noiseSourceAmplitude = 1.0f;
float ayg = 0.0f;
float azg = 0.0f;
float rxg = 0.0f;
float speedFactor = 0.7f;
float steerFactor = 1.0f;
float speedFilterConstant = 0.9f;
float steerFilterConstant = 0.9f;

char robotName[kRobotNameBufferSize] = {0};
char BTaddress[kBtAddressBufferSize] = {0};

esp_adc_cal_characteristics_t adcChars;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

extern fastStepper motLeft;
extern fastStepper motRight;

void IRAM_ATTR motLeftTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}

#ifdef esp32_s3
HardwareSerial& tmcSerialStream = Serial1;
TMC2209 stepperDriverLeft;
TMC2209 stepperDriverRight;

fastStepper motLeft(11, 10, 0, motLeftTimerFunction);
fastStepper motRight(14, 13, 1, motRightTimerFunction);
#else
fastStepper motLeft(5, 4, 0, motLeftTimerFunction);
#ifdef NEWPCB
fastStepper motRight(2, 15, 1, motRightTimerFunction);
#else
fastStepper motRight(2, 15, 1, motRightTimerFunction);
#endif
#endif

void echoChipInformation();
void initHardware();
void initStorage();
void initImu();
void initNetwork();
void initMqtt();
void initOta();
void initWebServer();
void initInput();
void restorePreferences();
void updateRemoteControlInputs();
void updateControlLoop(uint32_t nowMs);
void processActiveControl(uint32_t nowMs);
void processInactiveControl(uint32_t nowMs);
void applyDriveOutputs(float pidAngleOutput, float steer);
void publishBatteryStatus(uint32_t nowMs);
void publishPlotData();
void parseSerial();
void parseCommand(const char* data, uint8_t length);
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax);
void calculateGyroOffset(uint8_t nSample);
void readSensor();
void initSensor(uint8_t sampleCount);
void setMicroStep(uint8_t uStep);
void sendWifiList();
void initMqtt();
void mqttReconnect();
void publishRobotStatus();
void handleMqttMessage(char* topic, byte* payload, unsigned int length);
void handleIncomingCommand(const char* command);
const char* controlStateName();
const char* controlModeName();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendConfigurationData(uint8_t num);

#ifdef INPUT_PPM
volatile int32_t rxData[] = {0,0,0,0,0,0,0,0};
volatile uint32_t rxPre = 1;
volatile uint8_t channelNr = 0;
volatile uint8_t firstRoundCounter = 2;
volatile bool firstRoundPassed = false;
volatile bool validRxValues = false;

void rxFalling() {
  if (micros() - rxPre > 6000) {
    rxPre = micros();
    channelNr = 0;
    firstRoundCounter--;
    if (firstRoundCounter == 0) {
      firstRoundPassed = true;
    }
  } else {
    rxData[channelNr] = micros() - rxPre;
    rxPre = micros();
    channelNr++;
  }

  if (!validRxValues && firstRoundPassed) {
    validRxValues = true;
  }
}
#endif

void setup() {
  Serial.begin(115200);
  delay(50);

  echoChipInformation();

  Serial.println("setup: hardware");
  initHardware();
  Serial.println("setup: storage");
  initStorage();
  Serial.println("setup: imu");
  initImu();
  Serial.println("setup: preferences");
  restorePreferences();
  Serial.println("setup: network");
  initNetwork();
  Serial.println("setup: mqtt");
  initMqtt();
  Serial.println("setup: ota");
  initOta();
  Serial.println("setup: web server");
  initWebServer();
  Serial.println("setup: input");
  initInput();

#ifdef esp32_s3
  stepperDriverLeft.setHardwareEnablePin(kTmcEnablePinLeft);
  stepperDriverRight.setHardwareEnablePin(kTmcEnablePinRight);
  stepperDriverLeft.setup(tmcSerialStream, kTmcSerialBaudRateStep, TMC2209::SERIAL_ADDRESS_0, kTmcRxPin, kTmcTxPin);
  stepperDriverRight.setup(tmcSerialStream, kTmcSerialBaudRateStep, TMC2209::SERIAL_ADDRESS_1, kTmcRxPin, kTmcTxPin);
  stepperDriverLeft.enableAutomaticCurrentScaling();
  stepperDriverRight.enableAutomaticCurrentScaling();
  //stepperDriverLeft.enableAutomaticGradientAdaptation();
  //stepperDriverRight.enableAutomaticGradientAdaptation();
  setMicroStep(microStep);
  stepperDriverLeft.setRunCurrent(kTmcRunCurrent);
  stepperDriverRight.setRunCurrent(kTmcRunCurrent);
  pinMode(kPinFan, OUTPUT);
  digitalWrite(kPinFan, HIGH);
  if(stepperDriverLeft.isCommunicating() && stepperDriverRight.isCommunicating()) {
    Serial.println("stepperdrivers set up and communicating");
  } else {
    while(!stepperDriverLeft.isCommunicating() && !stepperDriverRight.isCommunicating()) {
      Serial.println("Stepper drivers not working, trying again");
      stepperDriverLeft.setup(tmcSerialStream, kTmcSerialBaudRateStep, TMC2209::SERIAL_ADDRESS_0, kTmcRxPin, kTmcTxPin);
      stepperDriverRight.setup(tmcSerialStream, kTmcSerialBaudRateStep, TMC2209::SERIAL_ADDRESS_1, kTmcRxPin, kTmcTxPin);
    }
  }
#endif

  motLeft.init();
  motRight.init();
  motLeft.microStep = microStep;
  motRight.microStep = microStep;

#ifdef esp32_s3
  stepperDriverLeft.enable();
  stepperDriverRight.enable();
#endif

#ifdef esp32_s3
  analogWrite(kPinMotorCurrent, motorCurrent);
#else
  dacWrite(kPinMotorCurrent, motorCurrent);
#endif
  // Initialize PID controllers. Angle PID uses preference-backed defaults loaded in restorePreferences().
  pidAngle.setParameters(pref_pid1_p, pref_pid1_i, pref_pid1_d, pref_pid1_m);
  pidPos.setParameters(1.0f, 0.0f, 1.2f, 50);
  pidSpeed.setParameters(6.0f, 5.0f, 0.0f, 20);

  Serial.println("Ready");

  esp_adc_cal_value_t adcType = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_0db, ADC_WIDTH_BIT_12, 1100, &adcChars);
  if (adcType == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.println("eFuse Vref");
  } else if (adcType == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.println("Two Point");
  } else {
    Serial.println("Default");
  }

  Serial << "ADC calibration values (attenuation, vref, coeff a, coeff b): "
         << adcChars.atten << "\t" << adcChars.vref << "\t"
         << adcChars.coeff_a << "\t" << adcChars.coeff_b << endl;

  adc1_config_channel_atten(kBatteryAdcChannel, ADC_ATTEN_0db);
#ifndef esp32_s3
  adc_set_data_inv(ADC_UNIT_1, true);
#endif

  Serial.println("Booted, ready for driving!");
  digitalWrite(kPinLedRight, HIGH);
}

void loop() {
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  static uint32_t lastControlUs = 0;
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  if (nowUs - lastControlUs >= kControlPeriodUs) {
    lastControlUs = nowUs;
    readSensor();
    updateRemoteControlInputs();
    updateControlLoop(nowMs);
    motLeft.update();
    motRight.update();
    publishBatteryStatus(nowMs);
    publishPlotData();
    parseSerial();
    ArduinoOTA.handle();
#ifdef INPUT_IBUS
    IBus.loop();
#endif
    wsServer.loop();
#ifdef INPUT_PS3
#ifndef esp32_s3
    if (Ps3.isConnected()) {
      remoteControl.speed = -1.0f * Ps3.data.analog.stick.ly / 1.27f * remoteControl.speedGain + remoteControl.speedOffset;
      remoteControl.steer = Ps3.data.analog.stick.rx / 1.27f * remoteControl.steerGain;
    }
#endif
#endif

    if (nowMs - lastMqttPublishMs >= kMqttPublishIntervalMs) {
      lastMqttPublishMs = nowMs;
      publishRobotStatus();
    }
  }
}

void echoChipInformation() {
  // Print ESP32 chip information to serial console
  Serial.println("\n--- Connected ESP Chip Information ---");

  // Built-in Arduino core helper
  Serial.printf("Chip Model:        %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision:     %d\n", ESP.getChipRevision());
  Serial.printf("CPU Frequency:     %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size:        %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  
  // Unique MAC / Chip ID
  uint64_t chipid = ESP.getEfuseMac();
  Serial.printf("Unique MAC/ID:     %04X%08X\n", (uint16_t)(chipid>>32), (uint32_t)chipid);
  Serial.println("--------------------------------------\n");
}

void initHardware() {
#ifndef esp32_s3
  Serial.println("hardware: motor enable");
  pinMode(kPinMotorEnable, OUTPUT);
#endif
#ifndef esp32_s3
  Serial.println("hardware: microstep pins");
  pinMode(kPinMicroStep1, OUTPUT);
  pinMode(kPinMicroStep2, OUTPUT);
  pinMode(kPinMicroStep3, OUTPUT);
#endif
#ifndef esp32_s3
  Serial.println("hardware: motor enable high");
  digitalWrite(kPinMotorEnable, HIGH);
#endif
#ifndef esp32_s3
  Serial.println("hardware: set microstep");
#endif
  setMicroStep(microStep);

  Serial.println("hardware: leds");
  pinMode(kPinLed, OUTPUT);
  pinMode(kPinLedLeft, OUTPUT);
  pinMode(kPinLedRight, OUTPUT);
  digitalWrite(kPinLed, LOW);
  digitalWrite(kPinLedLeft, HIGH);
  digitalWrite(kPinLedRight, LOW);
}

void initStorage() {
  preferences.begin("settings", false);
  if (!SPIFFS.begin(kFormatSPIFFSIfFailed)) {
    Serial.println("SPIFFS mount failed");
  } else {
    Serial.println("SPIFFS mount success");
  }
}

void initImu() {
  delay(200);
#ifdef esp32_s3
  Wire.setPins(kImuSdaPin, kImuSclPin);
  Wire.begin(kImuSdaPin, kImuSclPin, 400000UL);
  Wire.setTimeOut(20);
  delay(100);
  if (imu.begin() != 0) {
    Serial.println("LSM6DS3 initialization failed");
    return;
  } else {
    Serial.println("LSM6DS3 initialized");
  }
  imuReady = true;
  Serial.printf("LSM6DS3 sample: ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\n",
                imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ(),
                imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ());
#else
  Wire.begin(21, 22, 400000UL);
  delay(100);
  Serial.println(imu.testConnection());
  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
#endif
  delay(50);
  initSensor(50);
}

void restorePreferences() {
  if (preferences.getUInt("pref_version", 0) != kPreferenceVersion) {
    preferences.clear();
    preferences.putUInt("pref_version", kPreferenceVersion);
    Serial << "EEPROM init complete, all preferences deleted, new pref_version: " << kPreferenceVersion << "\n";
  }

  for (uint8_t i = 0; i < 3; ++i) {
    char key[16];
    sprintf(key, "gyro_offset_%u", i);
    gyroOffset[i] = preferences.getShort(key, 0);
    Serial << gyroOffset[i] << "\t";
  }
  Serial << endl;

  angleOffset = preferences.getFloat("angle_offset", 0.0f);

  // Load stored PID angle controller parameters if present
  pref_pid1_p = preferences.getFloat("pid1_p", pref_pid1_p);
  pref_pid1_i = preferences.getFloat("pid1_i", pref_pid1_i);
  pref_pid1_d = preferences.getFloat("pid1_d", pref_pid1_d);
  pref_pid1_m = preferences.getFloat("pid1_m", pref_pid1_m);

  size_t nameLength = preferences.getBytes("robot_name", robotName, sizeof(robotName));
  if (nameLength == 0) {
    strncpy(robotName, kDefaultRobotName, sizeof(robotName));
    robotName[sizeof(robotName) - 1] = '\0';
  } else {
    robotName[min(nameLength, sizeof(robotName) - 1)] = '\0';
  }

  Serial.println(robotName);
}

void initNetwork() {
  bool wifiConnected = false;
  if (preferences.getUInt("wifi_mode", 0) == 1) {
    char ssid[63] = {0};
    char key[63] = {0};
    preferences.getBytes("wifi_ssid", ssid, sizeof(ssid));
    preferences.getBytes("wifi_key", key, sizeof(key));
    Serial << "Connecting to '" << ssid << "'" << endl;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, key);
    if (WiFi.waitForConnectResult(10000) == WL_CONNECTED) {
      Serial.print("Connected to WiFi with IP address: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
    } else {
      Serial.println("Could not connect to known WiFi network");
    }
  }

  if (!wifiConnected) {
    Serial.println("Starting AP...");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(robotName, kWifiApPassword);
    Serial << "AP named '" << WiFi.softAPSSID() << "' started, IP address: " << WiFi.softAPIP() << endl;
  }
}

void initMqtt() {
  mqttClient.setServer(kMqttBroker, kMqttPort);
  mqttClient.setCallback(handleMqttMessage);
  snprintf(mqttStatusTopic, sizeof(mqttStatusTopic), "robots/%s/status", robotName);
  snprintf(mqttCommandTopic, sizeof(mqttCommandTopic), "robots/%s/command", robotName);
}

void mqttReconnect() {
  static uint32_t lastMqttAttemptMs = 0;

  if (mqttClient.connected()) {
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  if (millis() - lastMqttAttemptMs < 2000) {
    return;
  }
  lastMqttAttemptMs = millis();

  Serial << "Connecting to MQTT broker " << kMqttBroker << ":" << kMqttPort << "..." << endl;
  if (mqttClient.connect(robotName)) {
    Serial.println("MQTT connected");
    mqttClient.subscribe(mqttCommandTopic);
    mqttClient.subscribe(kMqttAllCommandTopic);
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqttClient.state());
  }
}

void publishRobotStatus() {
  if (!mqttClient.connected()) {
    return;
  }
  // Include PID angle controller parameters so external dashboards can initialize themselves
  char payload[512];
  snprintf(payload, sizeof(payload),
           "{\"name\":\"%s\",\"battery\":%.2f,\"angle\":%.2f,\"state\":\"%s\",\"mode\":\"%s\",\"c1p\":%.4f,\"c1i\":%.4f,\"c1d\":%.4f,\"c1m\":%.4f}",
           robotName,
           controlState.batteryVoltage,
           filterAngle,
           controlStateName(),
           controlModeName(),
           pidAngle.K,
           pidAngle.Ti,
           pidAngle.Td,
           pidAngle.maxOutput);
  mqttClient.publish(mqttStatusTopic, payload, true);
}

void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
  char message[64];
  size_t len = min(length, sizeof(message) - 1);
  memcpy(message, payload, len);
  message[len] = '\0';

  Serial << "MQTT msg on " << topic << ": " << message << endl;
  if (strcmp(topic, mqttCommandTopic) == 0 || strcmp(topic, kMqttAllCommandTopic) == 0) {
    // Support both simple named commands (stop, selfright) and the same
    // textual tuning/serial commands used by the web UI (e.g. "c1p0.65x").
    // If the payload starts with a tuning/control prefix we forward it to
    // the same parser used for WebSocket/serial commands. For other
    // short named commands, use handleIncomingCommand.
    if (message[0] == 'c' || message[0] == 'a' || message[0] == 'f' || message[0] == 'v' ||
        message[0] == 'm' || message[0] == 'u' || message[0] == 'g' || message[0] == 'p' ||
        message[0] == 'j' || message[0] == 'k' || message[0] == 'l' || message[0] == 'n' ||
        message[0] == 'w') {
      // parseCommand expects a terminating 'x' like the web UI sends.
      // If the MQTT payload doesn't include it, append it.
      size_t mlen = strlen(message);
      char buf[128];
      if (mlen > sizeof(buf) - 2) mlen = sizeof(buf) - 2;
      memcpy(buf, message, mlen);
      if (mlen == 0 || buf[mlen-1] != 'x') {
        buf[mlen] = 'x';
        buf[mlen+1] = '\0';
        parseCommand(buf, mlen+1);
      } else {
        buf[mlen] = '\0';
        parseCommand(buf, mlen);
      }
    } else {
      handleIncomingCommand(message);
    }
  }
}

void handleIncomingCommand(const char* command) {
  if (strcasecmp(command, "stop") == 0) {
    remoteControl.disableControl = true;
    Serial.println("MQTT command: stop");
  } else if (strcasecmp(command, "selfright") == 0 || strcasecmp(command, "restart") == 0) {
    remoteControl.selfRight = true;
    Serial.println("MQTT command: selfright/restart");
  } else if (strcasecmp(command, "store_pids") == 0) {
    // Store current PID angle controller parameters into preferences (EEPROM)
    preferences.putFloat("pid1_p", pidAngle.K);
    preferences.putFloat("pid1_i", pidAngle.Ti);
    preferences.putFloat("pid1_d", pidAngle.Td);
    preferences.putFloat("pid1_m", pidAngle.maxOutput);
    Serial << "Stored PID angle parameters to EEPROM: " << pidAngle.K << "\t" << pidAngle.Ti << "\t" << pidAngle.Td << "\t" << pidAngle.maxOutput << endl;
  }
}

const char* controlStateName() {
  if (controlState.selfRightActive) {
    return "selfright";
  }
  if (controlState.overrideActive) {
    return "override";
  }
  if (!controlState.active) {
    return "disabled";
  }
  return "active";
}

const char* controlModeName() {
  switch (controlMode) {
    case CONTROL_ANGLE_ONLY: return "angle_only";
    case CONTROL_ANGLE_POSITION: return "angle_position";
    case CONTROL_ANGLE_SPEED: return "angle_speed";
    default: return "unknown";
  }
}

void initOta() {
  ArduinoOTA.setHostname(robotName);
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  if (MDNS.begin(robotName)) {
    Serial.print("MDNS responder started, name: ");
    Serial.println(robotName);
  } else {
    Serial.println("Could not start MDNS responder");
  }
}

void initWebServer() {
  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("Loading index.htm");
    request->send(SPIFFS, "/index.htm");
  });

  httpServer.serveStatic("/", SPIFFS, "/");
  httpServer.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "FileNotFound");
  });

  httpServer.addHandler(new SPIFFSEditor(SPIFFS, kHttpUsername, kHttpPassword));
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(webSocketEvent);
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);
}

void initInput() {
#ifdef INPUT_IBUS
  IBus.begin(Serial2);
#endif
#ifdef INPUT_PPM
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), rxFalling, FALLING);
#endif
#ifdef INPUT_PS3
#ifndef esp32_s3
  Ps3.attach(onPs3Notify);
  Ps3.attachOnConnect(onPs3Connect);
  Ps3.attachOnDisconnect(onPs3Disconnect);
  Ps3.begin();
  String address = Ps3.getAddress();
  address.toCharArray(BTaddress, kBtAddressBufferSize);
  Serial.print("Bluetooth MAC address: ");
  Serial.println(address);
#endif
#endif
#ifdef esp32_s3
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowDataReceived);
  Serial.println("ESP-NOW controller receiver ready");
#endif
}

void updateRemoteControlInputs() {
#ifdef INPUT_IBUS
  if (IBus.isActive()) {
    remoteControl.speed = ((float)IBus.readChannel(1) - 1500.0f) / 5.0f * remoteControl.speedGain;
    remoteControl.steer = ((float)IBus.readChannel(0) - 1500.0f) / 5.0f * remoteControl.steerGain;

    bool selfRightInput = IBus.readChannel(3) > 1600 && IBus.readChannel(3) < 2100;
    bool disableInput = IBus.readChannel(3) < 1400 && IBus.readChannel(3) > 900;
    bool overrideInput = IBus.readChannel(4) > 1600;
    static bool lastSelfRight = false;
    static bool lastDisable = false;
    static bool lastOverride = false;

    remoteControl.selfRight = selfRightInput && !lastSelfRight;
    remoteControl.disableControl = disableInput && !lastDisable;
    remoteControl.overrideControl = overrideInput && !lastOverride;

    lastSelfRight = selfRightInput;
    lastDisable = disableInput;
    lastOverride = overrideInput;
  }
#endif

#ifdef INPUT_PPM
  if (rxData[1] == 0 || rxData[0] == 0) {
    remoteControl.speed = 0.0f;
    remoteControl.steer = 0.0f;
  } else {
    remoteControl.speed = mapFloat((float)constrain(rxData[1], minPPM, maxPPM), minPPM, maxPPM, -100.0f, 100.0f) * remoteControl.speedGain;
    remoteControl.steer = mapFloat((float)constrain(rxData[0], minPPM, maxPPM), minPPM, maxPPM, -100.0f, 100.0f) * remoteControl.steerGain;
  }
#endif

#ifdef esp32_s3
  static uint16_t lastButtons = 0;
  EspNowControllerPacket packet;
  memcpy(&packet, (const void*)&espNowControllerPacket, sizeof(packet));
  uint32_t lastPacketMs = espNowLastPacketMs;
  bool packetValid = espNowPacketReceived &&
                     millis() - lastPacketMs <= kEspNowControllerTimeoutMs &&
                     packet.connected;

  if (!packetValid) {
    remoteControl.speed = 0.0f;
    remoteControl.steer = 0.0f;
    lastButtons = 0;
  } else {
    remoteControl.speed = -1.0f * packet.leftStickY / 1.27f * remoteControl.speedGain + remoteControl.speedOffset;
    remoteControl.steer = packet.rightStickX / 1.27f * remoteControl.steerGain;

    uint16_t buttonDown = packet.buttons & ~lastButtons;
    if (buttonDown & ESP_NOW_BUTTON_DOWN) {
      remoteControl.speedGain = 0.05f;
      remoteControl.steerGain = 0.2f;
    }
    if (buttonDown & ESP_NOW_BUTTON_LEFT) {
      remoteControl.speedGain = 0.2f;
      remoteControl.steerGain = 0.4f;
    }
    if (buttonDown & ESP_NOW_BUTTON_UP) {
      remoteControl.speedGain = 0.3f;
      remoteControl.steerGain = 0.6f;
    }
    if (buttonDown & ESP_NOW_BUTTON_RIGHT) {
      remoteControl.speedGain = 0.7f;
      remoteControl.steerGain = 0.8f;
    }
    if (buttonDown & ESP_NOW_BUTTON_CIRCLE) remoteControl.selfRight = true;
    if (buttonDown & ESP_NOW_BUTTON_CROSS) remoteControl.disableControl = true;
    if (buttonDown & ESP_NOW_BUTTON_SQUARE) remoteControl.overrideControl = true;
    if (buttonDown & ESP_NOW_BUTTON_R1 && remoteControl.speedOffset < 20.0f) {
      remoteControl.speedOffset += 0.5f;
    }
    if (buttonDown & ESP_NOW_BUTTON_R2 && remoteControl.speedOffset > -20.0f) {
      remoteControl.speedOffset -= 0.5f;
    }
    lastButtons = packet.buttons;
  }
#endif
}

void updateControlLoop(uint32_t nowMs) {
  if (remoteControl.selfRight && !controlState.active) {
    controlState.selfRightActive = true;
    controlState.disableControlActive = false;
    remoteControl.selfRight = false;
  } else if (remoteControl.disableControl && controlState.active) {
    controlState.disableControlActive = true;
    controlState.selfRightActive = false;
    remoteControl.disableControl = false;
  }

  controlState.filteredSpeed = speedFilterConstant * controlState.filteredSpeed + (1.0f - speedFilterConstant) * remoteControl.speed / 5.0f;
  controlState.filteredSteer = steerFilterConstant * controlState.filteredSteer + (1.0f - steerFilterConstant) * remoteControl.steer;

  if (controlState.active) {
    processActiveControl(nowMs);
  } else {
    processInactiveControl(nowMs);
  }
}

void processActiveControl(uint32_t nowMs) {
  if (fabs(controlState.filteredSpeed) >= 0.2f) {
    controlState.lastInputMs = nowMs;
    if (controlMode == CONTROL_ANGLE_POSITION) {
      controlMode = CONTROL_ANGLE_SPEED;
      motLeft.setStep(0);
      motRight.setStep(0);
      pidSpeed.reset();
    }
  }

  if (controlMode == CONTROL_ANGLE_SPEED && nowMs - controlState.lastInputMs > 2000) {
    controlMode = CONTROL_ANGLE_POSITION;
    motLeft.setStep(0);
    motRight.setStep(0);
    pidPos.reset();
  }

  float pidPosOutput = 0.0f;
  float pidSpeedOutput = 0.0f;

  if (controlMode == CONTROL_ANGLE_ONLY) {
    pidAngle.setpoint = controlState.filteredSpeed * 2.0f;
  } else if (controlMode == CONTROL_ANGLE_POSITION) {
    int32_t averageStep = (motLeft.getStep() + motRight.getStep()) / 2;
    pidPos.setpoint = controlState.filteredSpeed;
    pidPos.input = -((float)averageStep) / 1000.0f;
    pidPosOutput = pidPos.calculate();
    pidAngle.setpoint = pidPosOutput;
  } else if (controlMode == CONTROL_ANGLE_SPEED) {
    pidSpeed.setpoint = controlState.filteredSpeed;
    pidSpeed.input = -controlState.averageMotorSpeed / 100.0f;
    pidSpeedOutput = pidSpeed.calculate();
    pidAngle.setpoint = pidSpeedOutput;
  }

  pidAngle.input = filterAngle;
  float pidAngleOutput = pidAngle.calculate();

  if (noiseSourceEnable) {
    float noiseValue = noiseSourceAmplitude * ((random(1000) / 1000.0f) - 0.5f);
    pidAngleOutput += noiseValue;
  }

  pidAngleOutput = min(pidAngleOutput, 25.0f);
  pidAngleOutput = max(pidAngleOutput, -25.0f);

  applyDriveOutputs(pidAngleOutput, controlState.filteredSteer);

  controlState.angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * (kControlPeriodUs / 1000000.0f);
  if (controlState.selfRightActive) {
    if (fabs(controlState.angleErrorIntegral) > kAngleErrorIntegralThresholdDuringSelfRight) {
      controlState.selfRightActive = false;
      controlState.disableControlActive = true;
    }
  } else if (fabs(controlState.angleErrorIntegral) > kAngleErrorIntegralThreshold) {
    controlState.disableControlActive = true;
  }

  if ((fabs(filterAngle) > kAngleDisableThreshold && !controlState.selfRightActive) || controlState.disableControlActive) {
    controlState.active = false;
    controlState.overrideActive = false;
    motLeft.speed = 0;
    motRight.speed = 0;
    digitalWrite(kPinMotorEnable, HIGH);
    digitalWrite(kPinLedLeft, LOW);
    digitalWrite(kPinLedRight, LOW);
  }

  if (fabs(filterAngle) < kAngleEnableThreshold && controlState.selfRightActive) {
    controlState.selfRightActive = false;
    controlState.angleErrorIntegral = 0;
  }
}

void applyDriveOutputs(float pidAngleOutput, float steer) {
  float nextAverageSpeed = controlState.averageMotorSpeed + pidAngleOutput / 2.0f;
  controlState.averageMotorSpeed = constrain(nextAverageSpeed, -maxStepSpeed, maxStepSpeed);

#ifdef esp32_s3
  motLeft.speed = controlState.averageMotorSpeed - steer;
  motRight.speed = -1.0*(controlState.averageMotorSpeed + steer);
#elif NEWPCB
  motLeft.speed = controlState.averageMotorSpeed - steer;
  motRight.speed = -1.0*(controlState.averageMotorSpeed + steer);
#else
  motLeft.speed = controlState.averageMotorSpeed + steer;
  motRight.speed = controlState.averageMotorSpeed - steer;
#endif

  uint8_t lastMicroStep = microStep;
  float absSpeed = fabs(controlState.averageMotorSpeed);
  if (absSpeed > (150.0f * 32.0f / microStep) && microStep > 1) {
    microStep /= 2;
  }
  if (absSpeed < (130.0f * 32.0f / microStep) && microStep < 32) {
    microStep *= 2;
  }
  if (microStep != lastMicroStep) {
    motLeft.microStep = microStep;
    motRight.microStep = microStep;
    setMicroStep(microStep);
  }
}

void processInactiveControl(uint32_t nowMs) {
  if (remoteControl.overrideControl) {
    remoteControl.overrideControl = false;
    motLeft.speed = 0;
    motRight.speed = 0;
    digitalWrite(kPinMotorEnable, LOW);
    controlState.overrideActive = true;
  }

  if (remoteControl.disableControl) {
    remoteControl.disableControl = false;
    digitalWrite(kPinMotorEnable, HIGH);
    controlState.overrideActive = false;
  }

  if (fabs(filterAngle) > kAngleEnableThreshold + 5.0f) {
    controlState.disableControlActive = false;
  }

  if ((fabs(filterAngle) < kAngleEnableThreshold || controlState.selfRightActive) && !controlState.disableControlActive) {
    controlState.active = true;
    digitalWrite(kPinLedLeft, HIGH);
    digitalWrite(kPinLedRight, HIGH);
    controlMode = CONTROL_ANGLE_POSITION;

    if (!controlState.overrideActive) {
      controlState.averageMotorSpeed = 0;
      digitalWrite(kPinMotorEnable, LOW);
      pidAngle.reset();
    } else {
      controlState.averageMotorSpeed = (motLeft.speed + motRight.speed) / 2.0f;
      controlState.overrideActive = false;
    }

    motLeft.setStep(0);
    motRight.setStep(0);
    pidPos.reset();
    pidSpeed.reset();
    controlState.angleErrorIntegral = 0;
  }

  if (controlState.overrideActive) {
    float speedCommand = controlState.filteredSpeed;
    float steerCommand = controlState.filteredSteer;
    motLeft.speed = -30.0f * speedCommand + 2.0f * steerCommand;
    motRight.speed = -30.0f * speedCommand - 2.0f * steerCommand;
    pidAngle.input = filterAngle;
    pidAngle.calculate();
  }
}

void publishBatteryStatus(uint32_t nowMs) {
  uint32_t rawReading = adc1_get_raw(kBatteryAdcChannel);
  if (rawReading >= 4096) {
    return;
  }
  uint32_t voltage = esp_adc_cal_raw_to_voltage(rawReading, &adcChars);
  controlState.batteryVoltage = controlState.batteryVoltage * kBatteryVoltageFilter + (voltage / 1000.0f) * kBatteryVoltageScale * (1.0f - kBatteryVoltageFilter);

  if (nowMs - controlState.lastBatteryReportMs > 5000 && wsServer.connectedClients(0) > 0) {
    char buffer[16];
    sprintf(buffer, "b%.1f", controlState.batteryVoltage);
    wsServer.broadcastTXT(buffer);
    controlState.lastBatteryReportMs = nowMs;
  }
}

void publishPlotData() {
  if (!plot.enable) {
    return;
  }

  if (++controlState.plotTicker < plot.prescaler) {
    return;
  }
  controlState.plotTicker = 0;

  // Validate that client 0 is actually connected before sending
  if (wsServer.connectedClients(0) == 0) {
    return;
  }

  union {
    struct {
      uint8_t cmd;
      uint8_t fill1;
      uint8_t fill2;
      uint8_t fill3;
      float f[13];
    } packet;
    uint8_t bytes[56];
  } plotData;

  plotData.packet.cmd = 255;
  plotData.packet.f[0] = micros() / 1000000.0f;
  plotData.packet.f[1] = accAngle;
  plotData.packet.f[2] = filterAngle;
  plotData.packet.f[3] = pidAngle.setpoint;
  plotData.packet.f[4] = pidAngle.input;
  plotData.packet.f[5] = pidAngle.calculate();
  plotData.packet.f[6] = pidPos.setpoint;
  plotData.packet.f[7] = pidPos.input;
  plotData.packet.f[8] = pidPos.calculate();
  plotData.packet.f[9] = pidSpeed.setpoint;
  plotData.packet.f[10] = pidSpeed.input;
  plotData.packet.f[11] = pidSpeed.calculate();
  plotData.packet.f[12] = 0.0f;

  if (!wsServer.sendBIN(0, plotData.bytes, sizeof(plotData.bytes))) {
    Serial.println("Warning: Failed to send plot data to client 0");
  }
}

void parseSerial() {
  static char serialBuf[64];
  static uint8_t pos = 0;

  while (Serial.available()) {
    char currentChar = Serial.read();
    if (pos < sizeof(serialBuf) - 1) {
      serialBuf[pos++] = currentChar;
    }
    if (currentChar == 'x') {
      parseCommand(serialBuf, pos);
      pos = 0;
      memset(serialBuf, 0, sizeof(serialBuf));
    }
  }
}

void parseCommand(const char* data, uint8_t length) {
  char buffer[64];
  if (length == 0) {
    return;
  }
  if (length >= sizeof(buffer)) {
    length = sizeof(buffer) - 1;
  }
  memcpy(buffer, data, length);
  buffer[length] = '\0';

  if (buffer[length - 1] != 'x') {
    return;
  }

  switch (buffer[0]) {
    case 'c': {
      uint8_t controllerNumber = buffer[1] - '0';
      char cmdType = buffer[2];
      float value = atof(buffer + 3);
      PID* target = nullptr;
      if (controllerNumber == 1) target = &pidAngle;
      else if (controllerNumber == 2) target = &pidPos;
      else if (controllerNumber == 3) target = &pidSpeed;
      if (target != nullptr) {
        switch (cmdType) {
          case 'p': target->K = value; break;
          case 'i': target->Ti = value; break;
          case 'd': target->Td = value; break;
          case 'n': target->N = value; break;
          case 't': target->controllerType = (uint8_t)value; break;
          case 'm': target->maxOutput = value; break;
          case 'o': target->minOutput = -value; break;
        }
        target->updateParameters();
        Serial << controllerNumber << "\t" << target->K << "\t" << target->Ti << "\t" << target->Td << "\t" << target->N << "\t" << target->controllerType << endl;
      }
      break;
    }
    case 'a':
      angleOffset = atof(buffer + 1);
      Serial << "Updating angle offset from " << preferences.getFloat("angle_offset");
      Serial << " to " << angleOffset << endl;
      preferences.putFloat("angle_offset", angleOffset);
      break;
    case 'f':
      gyroFilterConstant = atof(buffer + 1);
      Serial << gyroFilterConstant << endl;
      break;
    case 'v':
      motorCurrent = (uint8_t)atof(buffer + 1);
      Serial << motorCurrent << endl;
    #ifdef esp32_s3
      analogWrite(kPinMotorCurrent, motorCurrent);
    #else
      dacWrite(kPinMotorCurrent, motorCurrent);
    #endif
      break;
    case 'm':
      controlMode = static_cast<ControlMode>(atoi(buffer + 1));
      break;
    case 'u':
      microStep = atoi(buffer + 1);
      setMicroStep(microStep);
      break;
    case 'g':
      gyroGain = atof(buffer + 1);
      break;
    case 'p': {
      switch (buffer[1]) {
        case 'e': plot.enable = atoi(buffer + 2); break;
        case 'p': plot.prescaler = atoi(buffer + 2); break;
        case 'n': noiseSourceEnable = atoi(buffer + 2); break;
        case 'a': noiseSourceAmplitude = atof(buffer + 2); break;
      }
      break;
    }
    case 'j':
      gyroGain = atof(buffer + 1);
      break;
    case 'k': {
      uint8_t cmd = atoi(buffer + 1);
      if (cmd == 1) {
        calculateGyroOffset(100);
      } else if (cmd == 2) {
        Serial << "Updating angle offset from " << angleOffset;
        angleOffset = filterAngle;
        Serial << " to " << angleOffset << endl;
        preferences.putFloat("angle_offset", angleOffset);
      }
      break;
    }
    case 'l':
      maxStepSpeed = atof(buffer + 1);
      break;
    case 'n':
      gyroFilterConstant = atof(buffer + 1);
      break;
    case 'w': {
      char cmdType = buffer[1];
      char stringValue[64] = {0};
      switch (cmdType) {
        case 'r':
          Serial.println("Rebooting...");
          ESP.restart();
          break;
        case 'l':
          sendWifiList();
          break;
        case 's':
          strncpy(stringValue, buffer + 2, length - 3);
          preferences.putBytes("wifi_ssid", stringValue, kRobotNameBufferSize);
          Serial << "Updated WiFi SSID to: " << stringValue << endl;
          break;
        case 'k':
          strncpy(stringValue, buffer + 2, length - 3);
          preferences.putBytes("wifi_key", stringValue, kRobotNameBufferSize);
          Serial << "Updated WiFi key to: " << stringValue << endl;
          break;
        case 'm':
          preferences.putUInt("wifi_mode", atoi(buffer + 2));
          Serial << "Updated WiFi mode to (0=access point, 1=connect to SSID): " << atoi(buffer + 2) << endl;
          break;
        case 'n':
          strncpy(stringValue, buffer + 2, length - 3);
          if (strlen(stringValue) >= 8) {
            preferences.putBytes("robot_name", stringValue, kRobotNameBufferSize);
          }
          Serial << "Updated robot name to: " << stringValue << endl;
          break;
      }
      break;
    }
    default:
      break;
  }
}

void sendWifiList() {
  char buffer[200];
  size_t position = 2;
  buffer[0] = 'w';
  buffer[1] = 'l';
  Serial.println("Scan started");

  uint8_t networks = WiFi.scanNetworks();
  if (networks > 5) {
    networks = 5;
  }

  for (uint8_t i = 0; i < networks; ++i) {
    position += sprintf(buffer + position, "%s,", WiFi.SSID(i).c_str());
  }
  if (position > 2) {
    buffer[position - 1] = '\0';
  } else {
    buffer[position] = '\0';
  }

  Serial.println(buffer);
  wsServer.sendTXT(0, buffer);
}

void calculateGyroOffset(uint8_t nSample) {
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int16_t x, y, z;
  for (uint8_t i = 0; i < nSample; ++i) {
#ifdef esp32_s3
    x = (int16_t)(imu.readFloatGyroX() * kGyroSensitivity);
    y = (int16_t)(imu.readFloatGyroY() * kGyroSensitivity);
    z = (int16_t)(imu.readFloatGyroZ() * kGyroSensitivity);
#else
    imu.getRotation(&x, &y, &z);
#endif
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(5);
  }
  gyroOffset[0] = sumX / nSample;
  gyroOffset[1] = sumY / nSample;
  gyroOffset[2] = sumZ / nSample;
  for (uint8_t i = 0; i < 3; ++i) {
    char key[16];
    sprintf(key, "gyro_offset_%u", i);
    preferences.putShort(key, gyroOffset[i]);
  }
  Serial << "New gyro calibration values: " << gyroOffset[0] << "\t" << gyroOffset[1] << "\t" << gyroOffset[2] << endl;
}

void readSensor() {
#ifdef esp32_s3
  if (!imuReady) {
    return;
  }
#endif
  int16_t ax, ay, az, gx, gy, gz;
#ifdef esp32_s3
  ax = (int16_t)(imu.readFloatAccelX() * 16384.0f);
  ay = (int16_t)(imu.readFloatAccelY() * 16384.0f);
  az = (int16_t)(imu.readFloatAccelZ() * 16384.0f);
  gx = (int16_t)(imu.readFloatGyroX() * kGyroSensitivity);
  gy = (int16_t)(imu.readFloatGyroY() * kGyroSensitivity);
  gz = (int16_t)(imu.readFloatGyroZ() * kGyroSensitivity);
#else
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#endif
#ifdef esp32_s3
  accAngle = atan2f((float)ax, (float)ay) * 180.0f / M_PI - angleOffset;
  float deltaGyroAngle = ((float)(gz - gyroOffset[2]) / kGyroSensitivity) * (kControlPeriodUs / 1000000.0f) * gyroGain;
#else
  accAngle = atan2f((float)ay, (float)az) * 180.0f / M_PI - angleOffset;
  float deltaGyroAngle = ((float)(gx - gyroOffset[0]) / kGyroSensitivity) * (kControlPeriodUs / 1000000.0f) * gyroGain;
#endif
  filterAngle = gyroFilterConstant * (filterAngle + deltaGyroAngle) + (1.0f - gyroFilterConstant) * accAngle;
  ayg = (ay * 9.81f) / 16384.0f;
  azg = (az * 9.81f) / 16384.0f;
#ifdef esp32_s3
  rxg = ((float)(gz - gyroOffset[2]) / kGyroSensitivity);
#else
  rxg = ((float)(gx - gyroOffset[0]) / kGyroSensitivity);
#endif
}

void initSensor(uint8_t sampleCount) {
  float backup = gyroFilterConstant;
  gyroFilterConstant = 0.8f;
  for (uint8_t i = 0; i < sampleCount; ++i) {
    readSensor();
  }
  gyroFilterConstant = backup;
}

void setMicroStep(uint8_t uStep) {

  if (uStep == 0 || (uStep & (uStep - 1)) != 0) {
    return;
  }
#ifdef esp32_s3
  stepperDriverLeft.setMicrostepsPerStep(uStep);
  stepperDriverRight.setMicrostepsPerStep(uStep);
  motLeft.microStep = uStep;
  motRight.microStep = uStep;
#else
  uint8_t uStepPow = 0;
  uint8_t copy = uStep;
  while (copy >>= 1) {
    ++uStepPow;
  }

  digitalWrite(kPinMicroStep1, uStepPow & 0x01);
  digitalWrite(kPinMicroStep2, uStepPow & 0x02);
  digitalWrite(kPinMicroStep3, uStepPow & 0x04);

#ifdef STEPPER_DRIVER_A4988
  if (uStep == 16) {
    digitalWrite(kPinMicroStep1, HIGH);
    digitalWrite(kPinMicroStep2, HIGH);
    digitalWrite(kPinMicroStep3, HIGH);
  }
#endif
#endif
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = wsServer.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      sendConfigurationData(num);
      break;
    }
    case WStype_TEXT:
      parseCommand((const char*)payload, length);
      break;
    case WStype_BIN: {
      if (length == 6) {
        cmd message;
        memcpy(message.arr, payload, 6);
        Serial << "Binary: " << message.grp << "\t" << message.cmd << "\t" << message.val << "\t" << sizeof(cmd) << endl;
        if (message.grp == 100) {
          switch (message.cmd) {
            case 0: remoteControl.speed = message.val; break;
            case 1: remoteControl.steer = message.val; break;
            case 2: remoteControl.selfRight = true; break;
            case 3: remoteControl.disableControl = true; break;
          }
        }
      }
      break;
    }
    default:
      break;
  }
}

void sendConfigurationData(uint8_t num) {
  char packet[63];
  
  // Throttle config message burst to prevent socket buffer overflow
  // Send PID angle parameters
  sprintf(packet, "c%dp%.4f", 1, pidAngle.K);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%di%.4f", 1, pidAngle.Ti);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dd%.4f", 1, pidAngle.Td);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dn%.4f", 1, pidAngle.N);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dr%.4f", 1, pidAngle.R);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dm%.4f", 1, pidAngle.maxOutput);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%do%.4f", 1, -pidAngle.minOutput);
  wsServer.sendTXT(num, packet);
  yield();

  // Send PID position parameters
  sprintf(packet, "c%dp%.4f", 2, pidPos.K);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%di%.4f", 2, pidPos.Ti);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dd%.4f", 2, pidPos.Td);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dn%.4f", 2, pidPos.N);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dr%.4f", 2, pidPos.R);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dm%.4f", 2, pidPos.maxOutput);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%do%.4f", 2, -pidPos.minOutput);
  wsServer.sendTXT(num, packet);
  yield();

  // Send PID speed parameters
  sprintf(packet, "c%dp%.4f", 3, pidSpeed.K);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%di%.4f", 3, pidSpeed.Ti);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dd%.4f", 3, pidSpeed.Td);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dn%.4f", 3, pidSpeed.N);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dr%.4f", 3, pidSpeed.R);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%dm%.4f", 3, pidSpeed.maxOutput);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "c%do%.4f", 3, -pidSpeed.minOutput);
  wsServer.sendTXT(num, packet);
  yield();

  // Send filter and control parameters
  sprintf(packet, "h%.4f", speedFilterConstant);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "i%.4f", steerFilterConstant);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "v%d", motorCurrent);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "j%.4f", gyroGain);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "n%.4f", gyroFilterConstant);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "l%.4f", maxStepSpeed);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "wm%d", preferences.getUInt("wifi_mode", 0));
  wsServer.sendTXT(num, packet);
  yield();

  // Send WiFi and Bluetooth configuration
  char stringValue[63] = {0};
  preferences.getBytes("wifi_ssid", stringValue, sizeof(stringValue));
  sprintf(packet, "ws%s", stringValue);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "wn%s", robotName);
  wsServer.sendTXT(num, packet);
  yield();
  
  sprintf(packet, "wb%s", BTaddress);
  wsServer.sendTXT(num, packet);
  yield();
}

#if defined(INPUT_PS3) && !defined(esp32_s3)
void onPs3Notify() {
  if (Ps3.event.button_down.down) {
    remoteControl.speedGain = 0.05f;
    remoteControl.steerGain = 0.2f;
  }
  if (Ps3.event.button_down.left) {
    remoteControl.speedGain = 0.2f;
    remoteControl.steerGain = 0.4f;
  }
  if (Ps3.event.button_down.up) {
    remoteControl.speedGain = 0.3f;
    remoteControl.steerGain = 0.6f;
  }
  if (Ps3.event.button_down.right) {
    remoteControl.speedGain = 0.7f;
    remoteControl.steerGain = 0.8f;
  }
  if (Ps3.event.button_down.circle) remoteControl.selfRight = true;
  if (Ps3.event.button_down.cross) remoteControl.disableControl = true;
  if (Ps3.event.button_down.square) remoteControl.overrideControl = true;
  if (Ps3.event.button_down.r1) {
    if (remoteControl.speedOffset < 20.0f) {
      remoteControl.speedOffset += 0.5f;
    }
  }
  if (Ps3.event.button_down.r2) {
    if (remoteControl.speedOffset > -20.0f) {
      remoteControl.speedOffset -= 0.5f;
    }
  }
}

void onPs3Connect() {
  digitalWrite(kPinLed, HIGH);
  Serial.println("Bluetooth controller connected");
}

void onPs3Disconnect() {
  digitalWrite(kPinLed, LOW);
  Serial.println("Bluetooth controller disconnected");
  remoteControl.speed = 0;
  remoteControl.steer = 0;
  remoteControl.speedGain = 1.0f;
  remoteControl.steerGain = 1.0f;
}
#endif

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
