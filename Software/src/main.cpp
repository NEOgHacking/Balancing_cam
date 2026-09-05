#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <FastLED.h>
#include <TMC2209.h>
//#include <fastStepper.h>

#define NUM_LEDS 2

CRGB leds[NUM_LEDS];


// Define your custom ESP32-S3 hardware pins
#define I2C_SDA 37
#define I2C_SCL 38

// Instantiate the specific class for your sensor variant
Adafruit_LSM6DS3TRC lsm6ds; 


HardwareSerial & serial_stream = Serial1;

const uint8_t STEP_PIN = 11;
const uint8_t DIRECTION_PIN = 10;
const uint8_t EN_PIN = 9;
const uint32_t STEP_COUNT = 200;
const uint16_t HALF_STEP_DURATION_MICROSECONDS = 100;
const uint16_t STOP_DURATION = 1000;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 10;

// Instantiate TMC2209
TMC2209 stepper_driver;

void setup() {

    FastLED.addLeds<WS2812, 35>(leds, NUM_LEDS); 
    Serial.begin(115200);
    //while (!Serial) delay(10); // Wait for serial monitor

    Serial.println("Initializing Custom I2C Bus...");
    
    // Pass custom pins directly to the Wire initialization
    if (!Wire.begin(I2C_SDA, I2C_SCL, 400000)) { // 400kHz speed
        Serial.println("Failed to initialize I2C pins!");
        while (1) delay(10);
    }

    Serial.println("Connecting to LSM6DS3TR-C...");

    // Inject the initialized global &Wire object into the Adafruit library.
    // Default I2C address for LSM6DS3TR-C is usually 0x6A (or 0x6B depending on SA0 pin)
    if (!lsm6ds.begin_I2C(0x6B, &Wire)) { 
        Serial.println("LSM6DS3TR-C not found! Check wiring/pull-ups.");
        while (1) delay(10);
    }

    Serial.println("LSM6DS3TR-C successfully initialized!");

    // Optional: Configure sensor ranges
    //lsm6ds.setAccelRange(LSM6DS_ACCEL_4G);
    //lsm6ds.setGyroRange(LSM6DS_GYRO_500_DPS);

    stepper_driver.setup(serial_stream);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver.enableCoolStep();
    stepper_driver.enable();
    //digitalWrite(EN_PIN, LOW);

}

void loop() {
    // Read normalized data events from the sensor
    sensors_event_t accel, gyro, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);

    Serial.printf("Accel X: %.2f, Y: %.2f, Z: %.2f m/s^2\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    Serial.printf("Gyro  X: %.2f, Y: %.2f, Z: %.2f rad/s\n", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    Serial.printf("Temp %.2f", temp.temperature);

    leds[0] = CRGB::Red; FastLED.show(); delay(500);
    leds[0] = CRGB::Blue; FastLED.show();

    
    delay(500);
}
