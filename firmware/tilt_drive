/*
  Tilt-controlled two-motor drive (roll, like an airplane banking)
  Arduino Nano + MPU6050 + TB6612FNG (motors A and B)

  Wiring used by this sketch:
    MPU6050: VCC -> 5V, GND -> GND, SDA -> A4, SCL -> A5, AD0 -> GND (address 0x68)
    TB6612:  VCC -> 5V, GND -> GND, VM -> battery (after fuse + switch)
             STBY -> D8
             PWMA -> D5, AIN1 -> D7, AIN2 -> D6   (motor on AO1/AO2)
             PWMB -> D11, BIN1 -> D9, BIN2 -> D10 (motor on BO1/BO2)

  Power-saving and power-stability features:
    - 200 ms wait at start so the MPU6050 is awake before we talk to it
    - Speed ramps up and down gradually (no sudden current spikes that
      sag the battery rail), and reversing passes through zero first
    - Speed is capped (MAX_PWM) to protect the 3-6V TT motors
    - Motors fully off inside the dead zone, and the driver is put into
      standby after the robot has been level for a while
    - If the sensor stops answering, motors stop and the MPU is re-initialised
*/

#include <Wire.h>

// ---------- Pins ----------
const uint8_t PWMA = 5;
const uint8_t AIN2 = 6;
const uint8_t AIN1 = 7;
const uint8_t STBY = 8;
const uint8_t BIN1 = 9;
const uint8_t BIN2 = 10;
const uint8_t PWMB = 11;

// ---------- MPU6050 ----------
const uint8_t MPU_ADDR         = 0x68;   // AD0 tied to GND
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ---------- Tuning ----------
const float START_DEG      = 5.0;    // motors start when tilt exceeds this
const float STOP_DEG       = 3.0;    // ...and stop when tilt falls below this (hysteresis)
const float FULL_SPEED_DEG = 40.0;   // tilt at which speed reaches MAX_PWM
const int   MIN_PWM        = 80;     // lowest value that reliably starts a TT motor (tune)
const int   MAX_PWM        = 180;    // 180/255 * 8.4V = ~5.9V max, safe for 3-6V motors
const int   RAMP_STEP      = 10;     // max PWM change per loop (10 ms): smaller = gentler
const float ALPHA          = 0.2;    // low-pass filter strength (smaller = smoother, slower)
const int   DIRECTION      = 1;      // set to -1 if the robot drives the wrong way for your tilt
const bool  INVERT_A       = false;  // flip if motor A's wheel spins the wrong way
const bool  INVERT_B       = false;  // flip if motor B's wheel spins the wrong way
const unsigned long IDLE_MS = 1000;  // level this long -> driver goes to standby
const bool  DEBUG          = true;   // Serial output; set false for battery-only runs

// ---------- State ----------
float filteredRoll = 0.0;
bool  filterReady  = false;
bool  active       = false;   // are we past the dead zone?
bool  standby      = true;    // is the driver in standby?
int   currentSpeed = 0;       // what the motors are doing right now
uint8_t failCount  = 0;
unsigned long idleSince = 0;
unsigned long lastPrint = 0;

// ---------- Motors ----------
// speed: -255..255. Positive = forward, negative = backward, 0 = stop (coast).
void driveMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void applySpeed(int speed) {
  if (speed != 0) {
    if (standby) {
      digitalWrite(STBY, HIGH);   // wake the driver before commanding it
      standby = false;
    }
    idleSince = millis();
  }

  driveMotor(AIN1, AIN2, PWMA, INVERT_A ? -speed : speed);
  driveMotor(BIN1, BIN2, PWMB, INVERT_B ? -speed : speed);

  if (speed == 0 && !standby && (millis() - idleSince >= IDLE_MS)) {
    digitalWrite(STBY, LOW);      // saves a little battery while sitting level
    standby = true;
  }
}

void stopNow() {
  currentSpeed = 0;
  applySpeed(0);
}

// Move currentSpeed toward target by at most RAMP_STEP.
void rampToward(int target) {
  if (currentSpeed < target) {
    currentSpeed = min(currentSpeed + RAMP_STEP, target);
  } else if (currentSpeed > target) {
    currentSpeed = max(currentSpeed - RAMP_STEP, target);
  }
}

// ---------- MPU6050 ----------
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool initMPU() {
  if (!writeReg(REG_PWR_MGMT_1, 0x00)) return false;   // wake from sleep
  delay(10);
  if (!writeReg(REG_CONFIG, 0x03)) return false;       // built-in low-pass (~44 Hz) against motor vibration
  return true;
}

int16_t readInt16() {
  // Two reads in separate statements so the byte order is guaranteed.
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
  int16_t ay = readInt16();
  int16_t az = readInt16();

  // Gravity always shows up on some axis, so all zeros means the MPU lost power/reset.
  if (ay == 0 && az == 0) return false;

  // Roll = rotation about the X axis (airplane banking).
  roll = atan2((float)ay, (float)az) * 180.0 / PI;
  return true;
}

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, LOW);     // driver off until we need it
  driveMotor(AIN1, AIN2, PWMA, 0);
  driveMotor(BIN1, BIN2, PWMB, 0);

  Serial.begin(115200);

  delay(200);                  // give the MPU6050 time to power up after switch-on

  Wire.begin();
  Wire.setClock(400000);

  // Keep trying until the MPU answers (motors stay off meanwhile)
  bool warned = false;
  while (!initMPU()) {
    if (!warned) {
      Serial.println(F("Waiting for MPU6050... check SDA/SCL, power, AD0 to GND."));
      warned = true;
    }
    delay(100);
  }
  Serial.println(F("MPU6050 ready."));
}

void loop() {
  float roll;
  if (!readRoll(roll)) {
    stopNow();                 // lost the sensor: stop the motors
    filterReady = false;
    active = false;
    failCount++;
    if (failCount >= 5) {      // several failures in a row: try to wake the MPU again
      initMPU();
      failCount = 0;
    }
    delay(20);
    return;
  }
  failCount = 0;

  // Smooth out vibration noise
  if (!filterReady) {
    filteredRoll = roll;
    filterReady = true;
  } else {
    filteredRoll += ALPHA * (roll - filteredRoll);
  }

  float angle = filteredRoll * DIRECTION;
  float mag = fabs(angle);

  // Dead zone with hysteresis so the motors don't chatter at the edge
  if (!active && mag > START_DEG) {
    active = true;
  } else if (active && mag < STOP_DEG) {
    active = false;
  }

  int target = 0;
  if (active) {
    float t = (mag - START_DEG) / (FULL_SPEED_DEG - START_DEG);
    t = constrain(t, 0.0, 1.0);
    int pwm = MIN_PWM + (int)(t * (MAX_PWM - MIN_PWM));
    target = (angle > 0) ? pwm : -pwm;
  }

  rampToward(target);
  applySpeed(currentSpeed);

  if (DEBUG && (millis() - lastPrint >= 200)) {
    lastPrint = millis();
    Serial.print(F("roll: "));
    Serial.print(filteredRoll, 1);
    Serial.print(F("  target: "));
    Serial.print(target);
    Serial.print(F("  speed: "));
    Serial.println(currentSpeed);
  }

  delay(10);
}
