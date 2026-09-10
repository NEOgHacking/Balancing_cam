#include "LSM6DS3TRC.h"

namespace {
constexpr uint8_t kWhoAmI = 0x0F;
constexpr uint8_t kCtrl1Xl = 0x10;
constexpr uint8_t kCtrl2G = 0x11;
constexpr uint8_t kCtrl3C = 0x12;
constexpr uint8_t kOutXl = 0x28;
constexpr uint8_t kOutXg = 0x22;
constexpr float kAccelScaleG = 0.000061f;
constexpr float kGyroScaleDps = 0.00875f;
}

LSM6DS3::LSM6DS3(LSM6DS3Interface, uint8_t address) : address_(address) {}

uint8_t LSM6DS3::begin() {
  Wire.begin();
  if (!probeAddress(address_)) {
    uint8_t alternateAddress = address_ == 0x6A ? 0x6B : 0x6A;
    if (!probeAddress(alternateAddress)) {
      return 1;
    }
    address_ = alternateAddress;
  }
  if (!writeRegister(kCtrl1Xl, 0x50) || !writeRegister(kCtrl2G, 0x50) || !writeRegister(kCtrl3C, 0x44)) {
    return 1;
  }
  return 0;
}

bool LSM6DS3::probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(kWhoAmI);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, (uint8_t)1) != 1) {
    return 1;
  }
  return Wire.read() == 0x6A;
}

bool LSM6DS3::writeRegister(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(address_);
  Wire.write(registerAddress);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int16_t LSM6DS3::readAxis(uint8_t registerAddress) {
  Wire.beginTransmission(address_);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address_, (uint8_t)2) != 2) {
    return 0;
  }
  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  return (int16_t)((uint16_t)high << 8 | low);
}

float LSM6DS3::readFloatAccelX() { return readAxis(kOutXl) * kAccelScaleG; }
float LSM6DS3::readFloatAccelY() { return readAxis(kOutXl + 2) * kAccelScaleG; }
float LSM6DS3::readFloatAccelZ() { return readAxis(kOutXl + 4) * kAccelScaleG; }
float LSM6DS3::readFloatGyroX() { return readAxis(kOutXg) * kGyroScaleDps; }
float LSM6DS3::readFloatGyroY() { return readAxis(kOutXg + 2) * kGyroScaleDps; }
float LSM6DS3::readFloatGyroZ() { return readAxis(kOutXg + 4) * kGyroScaleDps; }
