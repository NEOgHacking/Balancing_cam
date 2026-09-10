#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

// Define your custom I2C pins for the ESP32-S3
#define I2C_SDA 37
#define I2C_SCL 38

// Define the custom I2C address
#define LSM6DS_ADDRESS 0x6B

Adafruit_LSM6DS3TRC lsm6ds;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("LSM6DS3TR-C Custom Pin & Address Test");

  // 1. Initialize the I2C bus using the custom SDA and SCL pins
  bool i2c_initialized = Wire.begin(I2C_SDA, I2C_SCL);
  if (!i2c_initialized) {
    Serial.println("Failed to initialize I2C bus on pins 37 and 38");
    while (1) delay(10);
  }

  // 2. Initialize the sensor passing the specific address and Wire instance
  if (!lsm6ds.begin_I2C(LSM6DS_ADDRESS, &Wire)) {
    Serial.println("Failed to find LSM6DS3TR-C chip at address 0x6B");
    while (1) { delay(10); }
  }

  Serial.println("LSM6DS3TR-C Found successfully!");
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  
  // Fetch new data from the sensor
  lsm6ds.getEvent(&accel, &gyro, &temp);

  Serial.printf("Accel X: %.2f \tY: %.2f \tZ: %.2f m/s^2\n", 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  Serial.printf("Gyro  X: %.2f \tY: %.2f \tZ: %.2f rad/s\n", 
                gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
  Serial.printf("Temp   : %.2f C\n", temp.temperature);
  Serial.println();

  delay(500);
}
