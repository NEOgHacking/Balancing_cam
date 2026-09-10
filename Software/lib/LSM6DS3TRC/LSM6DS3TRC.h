#ifndef LSM6DS3TRC_H
#define LSM6DS3TRC_H

#include <Arduino.h>
#include <Wire.h>

enum LSM6DS3Interface : uint8_t {
  I2C_MODE = 0,
};

class LSM6DS3 {
 public:
  LSM6DS3(LSM6DS3Interface interfaceMode, uint8_t address);
  uint8_t begin();
  float readFloatAccelX();
  float readFloatAccelY();
  float readFloatAccelZ();
  float readFloatGyroX();
  float readFloatGyroY();
  float readFloatGyroZ();

 private:
  uint8_t address_;
  bool probeAddress(uint8_t address);
  int16_t readAxis(uint8_t registerAddress);
  bool writeRegister(uint8_t registerAddress, uint8_t value);
};

#endif
