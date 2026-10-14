/*
  Motor-only test: no MPU6050 involved.
  Runs motor A forward 2 s, stops 1 s, backward 2 s, stops 1 s, repeating.
  Needs the battery connected (VM) and the LM2596 set to 5.0V.
*/

const uint8_t PWMA = 5;
const uint8_t AIN2 = 6;
const uint8_t AIN1 = 7;
const uint8_t STBY = 8;

const int TEST_PWM = 150;   // ~4.4V at 7.4V battery

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);   // driver must be taken out of standby
}

void loop() {
  // forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, TEST_PWM);
  delay(2000);

  // stop
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  delay(1000);

  // backward
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, TEST_PWM);
  delay(2000);

  // stop
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  delay(1000);
}
