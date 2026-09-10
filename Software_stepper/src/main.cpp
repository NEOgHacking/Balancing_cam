#include <TMC2209.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

HardwareSerial & serial_stream = Serial1;

const int RX_PIN = 18;
const int TX_PIN = 17;
const int L_STEP_PIN = 11;
const int R_STEP_PIN = 14;
const int L_DIR_PIN = 10;
const int R_DIR_PIN = 13;
const int L_EN_PIN = 9;
const int R_EN_PIN = 12;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 3000;

// Instantiate TMC2209
TMC2209 stepper_driver_0;
TMC2209 stepper_driver_1;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(L_STEP_PIN, OUTPUT);
  pinMode(R_STEP_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  stepper_driver_0.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, RX_PIN, TX_PIN);
  stepper_driver_1.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_1, RX_PIN, TX_PIN);
  stepper_driver_0.setHardwareEnablePin(L_EN_PIN);
  stepper_driver_1.setHardwareEnablePin(R_EN_PIN);
  stepper_driver_0.enableAutomaticCurrentScaling();
  stepper_driver_1.enableAutomaticCurrentScaling();
  stepper_driver_0.setRunCurrent(20);
  stepper_driver_1.setRunCurrent(20);
  stepper_driver_0.setMicrostepsPerStep(8);
  stepper_driver_1.setMicrostepsPerStep(8);
  stepper_driver_0.disable();
  stepper_driver_1.disable();
  if(!stepper_driver_0.isSetupAndCommunicating()) {
    Serial.println("Trying connection to stepper driver 0 again");
    stepper_driver_0.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, RX_PIN, TX_PIN);
    delay(100);
  }
  if(!stepper_driver_1.isSetupAndCommunicating()) {
    Serial.println("Trying connection to stepper driver 1 again");
    stepper_driver_1.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_1, RX_PIN, TX_PIN);
    delay(100);
  }
}

void loop()
{
  if (stepper_driver_0.isSetupAndCommunicating() && stepper_driver_1.isSetupAndCommunicating())
  {
    Serial.println("Stepper drivers 0 and 1 are setup and communicating!");
    Serial.println("Try turning driver power off to see what happens.");
  }
  else if (stepper_driver_0.isCommunicatingButNotSetup())
  {
    Serial.println("Stepper driver 0 is communicating but not setup!");
    Serial.println("Running setup again...");
    stepper_driver_0.setup(serial_stream);
  }
  else if (stepper_driver_1.isCommunicatingButNotSetup())
  {
    Serial.println("Stepper driver 1 is communicating but not setup!");
    Serial.println("Running setup again...");
    stepper_driver_1.setup(serial_stream);
  }
  else
  {
    Serial.println("Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
  }
  Serial.println();
  delay(DELAY);

  stepper_driver_0.enable();
  stepper_driver_1.enable();
  for(int i = 0; i < 2; i++) {
    if(i < 1) {
      digitalWrite(L_DIR_PIN, LOW);
      digitalWrite(R_DIR_PIN, LOW);
    }
    else {
      digitalWrite(L_DIR_PIN, HIGH);
      digitalWrite(R_DIR_PIN, HIGH);
    }
    for(int j = 0; j < 400; j++) {
      int step = j % 2;
      digitalWrite(L_STEP_PIN, j%2);
      digitalWrite(R_STEP_PIN, j%2);
      //Serial.print(">Step:"); Serial.print(step); Serial.print(", J:"); Serial.println(j);
      delay(5);
    }
  }


  Serial.println("*************************");
  Serial.println("getStatus()");
  TMC2209::Status status = stepper_driver_1.getStatus();
  Serial.print("status.over_temperature_warning = ");
  Serial.println(status.over_temperature_warning);
  Serial.print("status.over_temperature_shutdown = ");
  Serial.println(status.over_temperature_shutdown);
  Serial.print("status.short_to_ground_a = ");
  Serial.println(status.short_to_ground_a);
  Serial.print("status.short_to_ground_b = ");
  Serial.println(status.short_to_ground_b);
  Serial.print("status.low_side_short_a = ");
  Serial.println(status.low_side_short_a);
  Serial.print("status.low_side_short_b = ");
  Serial.println(status.low_side_short_b);
  Serial.print("status.open_load_a = ");
  Serial.println(status.open_load_a);
  Serial.print("status.open_load_b = ");
  Serial.println(status.open_load_b);
  Serial.print("status.over_temperature_120c = ");
  Serial.println(status.over_temperature_120c);
  Serial.print("status.over_temperature_143c = ");
  Serial.println(status.over_temperature_143c);
  Serial.print("status.over_temperature_150c = ");
  Serial.println(status.over_temperature_150c);
  Serial.print("status.over_temperature_157c = ");
  Serial.println(status.over_temperature_157c);
  Serial.print("status.current_scaling = ");
  Serial.println(status.current_scaling);
  Serial.print("status.stealth_chop_mode = ");
  Serial.println(status.stealth_chop_mode);
  Serial.print("status.standstill = ");
  Serial.println(status.standstill);
  Serial.println("*************************");
  Serial.println();

  Serial.println();

  stepper_driver_0.disable();
  stepper_driver_1.disable();
}
