/*
  Tilt-controlled single motor (roll, like an airplane banking)
  Arduino Nano + MPU6050 + TB6612FNG (motor A only)

  Wiring used by this sketch:
    MPU6050: VCC -> 5V, GND -> GND, SDA -> A4, SCL -> A5, AD0 -> GND (address 0x68)
    TB6612:  VCC -> 5V, GND -> GND, VM -> battery (7.4V), STBY -> D8
             PWMA -> D5, AIN1 -> D7, AIN2 -> D6
             Motor wires -> AO1 and AO2

  Behavior:
    Tilt one way  -> motor forwards (speed grows with tilt angle)
    Tilt other way -> motor backwards
    Near level (inside the dead zone) -> motor stops
*/

#include <Wire.h>

// ---------- Pins ----------
const uint8_t PWMA = 5;   // PWM-capable
const uint8_t AIN2 = 6;
const uint8_t AIN1 = 7;
const uint8_t STBY = 8;

// ---------- MPU6050 ----------
const uint8_t MPU_ADDR = 0x68;   // AD0 tied to GND
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ---------- Tuning ----------
const float DEADZONE_DEG   = 5.0;    // no movement inside +/- this angle
const float FULL_SPEED_DEG = 40.0;   // angle at which speed reaches MAX_PWM
const int   MIN_PWM        = 80;     // lowest value that reliably starts a TT motor (tune)
const int   MAX_PWM        = 180;    // 180/255 * 8.4V = ~5.9V max, safe for 3-6V TT motors
const int   DIRECTION      = 1;      // set to -1 if the motor runs the wrong way for your tilt
const float ALPHA          = 0.2;    // low-pass filter strength (smaller = smoother, slower)

float filteredRoll = 0.0;
bool  filterReady  = false;
unsigned long lastPrint = 0;

// ---------- Motor ----------
// speed: -255..255. Positive = forward, negative = backward, 0 = stop.
void setMotor(int speed) {
  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

// ---------- MPU6050 ----------
int16_t readInt16() {
  // Read the two bytes in separate statements so the order is guaranteed.
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

// Returns true and fills roll (degrees) on success.
bool readRoll(float &roll) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true) != 6) return false;

  readInt16();                 // accel X (not needed for roll, but must be read)
  int16_t ay = readInt16();    // accel Y
  int16_t az = readInt16();    // accel Z

  // Roll = rotation about the X axis (airplane banking).
  roll = atan2((float)ay, (float)az) * 180.0 / PI;
  return true;
}

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  setMotor(0);
  digitalWrite(STBY, HIGH);    // driver stays off unless STBY is HIGH

  Wire.begin();
  Wire.setClock(400000);

  // Wake the MPU6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("MPU6050 not found. Check SDA/SCL, power, and AD0 to GND."));
    setMotor(0);
    while (true) { }           // stop here with the motor off
  }
  Serial.println(F("MPU6050 ready."));
}

void loop() {
  float roll;
  if (!readRoll(roll)) {
    setMotor(0);               // lost the sensor: stop the motor
    return;
  }

  // Smooth out vibration noise
  if (!filterReady) {
    filteredRoll = roll;
    filterReady = true;
  } else {
    filteredRoll += ALPHA * (roll - filteredRoll);
  }

  float angle = filteredRoll * DIRECTION;
  float mag = fabs(angle);

  int speed = 0;
  if (mag > DEADZONE_DEG) {
    float t = (mag - DEADZONE_DEG) / (FULL_SPEED_DEG - DEADZONE_DEG);
    t = constrain(t, 0.0, 1.0);
    int pwm = MIN_PWM + (int)(t * (MAX_PWM - MIN_PWM));
    speed = (angle > 0) ? pwm : -pwm;
  }
  setMotor(speed);

  // Print ~10 times per second for debugging
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(F("roll: "));
    Serial.print(filteredRoll, 1);
    Serial.print(F("  speed: "));
    Serial.println(speed);
  }

  delay(10);
}
