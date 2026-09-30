/*
 * boat_motor_controller.ino
 *
 * Arduino UNO R4 WiFi firmware. Drives two BTS7960 (IBT-2) motor drivers
 * from serial commands sent by motor_driver.py on the Raspberry Pi, and
 * streams the Modulino Movement IMU back to the Pi.
 *
 * Serial protocol, 115200 baud, newline terminated:
 *
 *   Pi -> Arduino   "L:<int> R:<int>"   thrust in percent, -100..100
 *   Arduino -> Pi   "STATUS L_IS:<raw> R_IS:<raw> WDT:<0|1> L:<int> R:<int> TX_FREE:<n>"   2 Hz
 *                   "IMU AX:<g> AY:<g> AZ:<g> GR:<dps> GP:<dps> GY:<dps>"                 50 Hz
 *
 * Lines that don't parse are ignored and do not feed the watchdog.
 * L_IS/R_IS are raw analogRead values from the current-sense pins, not
 * amps. GR/GP/GY are gyro rates in deg/s, not angles; the Pi integrates them.
 *
 * Watchdog: if no valid command arrives for 500 ms, both motors are set to
 * zero and all enable pins go LOW. This runs on the Arduino so the motors
 * stop even if the Pi, ROS or the USB link dies.
 *
 * Motors are brushed DC thrusters, so they are driven with PWM duty on the
 * BTS7960 rather than an RC ESC.
 */

#include <Arduino.h>
#include <Arduino_Modulino.h>

ModulinoMovement movement;

const unsigned long IMU_PERIOD_MS = 20;
unsigned long lastImuMillis = 0;

// Pin map, verified on the bench 2026-09-30.
// Left motor BTS7960
const uint8_t PIN_L_RPWM = 5;
const uint8_t PIN_L_LPWM = 6;
const uint8_t PIN_L_R_EN = 2;
const uint8_t PIN_L_L_EN = 4;
const uint8_t PIN_L_R_IS = A0;
const uint8_t PIN_L_L_IS = A1;

// Right motor BTS7960
const uint8_t PIN_R_RPWM = 9;
const uint8_t PIN_R_LPWM = 10;
const uint8_t PIN_R_R_EN = 7;
const uint8_t PIN_R_L_EN = 8;
const uint8_t PIN_R_R_IS = A2;
const uint8_t PIN_R_L_IS = A3;

// RPWM/LPWM must stay on PWM-capable pins.

// Same timeout as motor_driver.py on the Pi side.
const unsigned long COMMAND_TIMEOUT_MS = 500;

const unsigned long STATUS_PERIOD_MS = 500;

const long SERIAL_BAUD = 115200;

// A valid command is under 20 characters.
const uint8_t MAX_LINE_LEN = 32;

char lineBuf[MAX_LINE_LEN];
uint8_t lineLen = 0;
unsigned long lastCommandMillis = 0;
unsigned long lastStatusMillis = 0;
bool watchdogTripped = true;  // stopped until the first valid command
int currentLeft = 0;   // last applied values, for STATUS
int currentRight = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_L_RPWM, OUTPUT);
  pinMode(PIN_L_LPWM, OUTPUT);
  pinMode(PIN_L_R_EN, OUTPUT);
  pinMode(PIN_L_L_EN, OUTPUT);

  pinMode(PIN_R_RPWM, OUTPUT);
  pinMode(PIN_R_LPWM, OUTPUT);
  pinMode(PIN_R_R_EN, OUTPUT);
  pinMode(PIN_R_L_EN, OUTPUT);

  stopAllMotors(/*dropEnable=*/true);

  // IMU on the Qwiic connector. Motor control still works if it is missing.
  Modulino.begin();
  movement.begin();

  lastCommandMillis = millis();
  lastStatusMillis = millis();
  lastImuMillis = millis();
}

void loop() {
  readSerialCommands();
  checkWatchdog();
  maybeSendStatus();
  maybeSendImu();
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
      continue;
    }
    if (c == '\r') {
      continue;
    }
    if (lineLen < MAX_LINE_LEN - 1) {
      lineBuf[lineLen++] = c;
    } else {
      // Overlong line is noise. Drop it and resync on the next newline.
      lineLen = 0;
    }
  }
}

void handleLine(const char *line) {
  int left, right;
  int parsed = sscanf(line, "L:%d R:%d", &left, &right);
  if (parsed != 2) {
    return;
  }

  left = clampThrust(left);
  right = clampThrust(right);

  applyThrust(left, right);
  currentLeft = left;
  currentRight = right;
  lastCommandMillis = millis();
  watchdogTripped = false;
}

int clampThrust(int v) {
  if (v > 100) return 100;
  if (v < -100) return -100;
  return v;
}

void checkWatchdog() {
  if (watchdogTripped) {
    return;
  }
  unsigned long elapsed = millis() - lastCommandMillis;
  if (elapsed > COMMAND_TIMEOUT_MS) {
    stopAllMotors(/*dropEnable=*/true);
    watchdogTripped = true;
    currentLeft = 0;
    currentRight = 0;
  }
}

// Positive drives RPWM (forward), negative drives LPWM (reverse).
// Enables go LOW while duty and direction change, and stay LOW for zero.
void applyMotor(int value, uint8_t rpwmPin, uint8_t lpwmPin,
                uint8_t rEnPin, uint8_t lEnPin) {
  digitalWrite(rEnPin, LOW);
  digitalWrite(lEnPin, LOW);

  // Linear map for now. Add a minimum duty here if the motors show a dead zone.
  uint8_t duty = (uint8_t)((abs(value) * 255L) / 100);

  if (value >= 0) {
    analogWrite(lpwmPin, 0);
    analogWrite(rpwmPin, duty);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, duty);
  }
  if (value != 0) {
    digitalWrite(rEnPin, HIGH);
    digitalWrite(lEnPin, HIGH);
  }
}

void applyThrust(int left, int right) {
  applyMotor(left, PIN_L_RPWM, PIN_L_LPWM, PIN_L_R_EN, PIN_L_L_EN);
  applyMotor(right, PIN_R_RPWM, PIN_R_LPWM, PIN_R_R_EN, PIN_R_L_EN);
}

// dropEnable also pulls the enable pins LOW, which disables the bridge
// regardless of the PWM signal.
void stopAllMotors(bool dropEnable) {
  if (dropEnable) {
    digitalWrite(PIN_L_R_EN, LOW);
    digitalWrite(PIN_L_L_EN, LOW);
    digitalWrite(PIN_R_R_EN, LOW);
    digitalWrite(PIN_R_L_EN, LOW);
  }
  analogWrite(PIN_L_RPWM, 0);
  analogWrite(PIN_L_LPWM, 0);
  analogWrite(PIN_R_RPWM, 0);
  analogWrite(PIN_R_LPWM, 0);
}

void maybeSendStatus() {
  unsigned long now = millis();
  if (now - lastStatusMillis < STATUS_PERIOD_MS) {
    return;
  }
  lastStatusMillis = now;

  // Only the R_IS pin of each module is read for now.
  int lIs = analogRead(PIN_L_R_IS);
  int rIs = analogRead(PIN_R_R_IS);

  Serial.print("STATUS L_IS:");
  Serial.print(lIs);
  Serial.print(" R_IS:");
  Serial.print(rIs);
  Serial.print(" WDT:");
  Serial.print(watchdogTripped ? 1 : 0);
  Serial.print(" L:"); Serial.print(currentLeft);
  Serial.print(" R:"); Serial.print(currentRight);
  Serial.print(" TX_FREE:"); Serial.println(Serial.availableForWrite());
}

void maybeSendImu() {
  unsigned long now = millis();
  if (now - lastImuMillis < IMU_PERIOD_MS) {
    return;
  }
  lastImuMillis = now;

  movement.update();

  Serial.print("IMU AX:");
  Serial.print(movement.getX(), 4);
  Serial.print(" AY:");
  Serial.print(movement.getY(), 4);
  Serial.print(" AZ:");
  Serial.print(movement.getZ(), 4);
  Serial.print(" GR:");
  Serial.print(movement.getRoll(), 3);
  Serial.print(" GP:");
  Serial.print(movement.getPitch(), 3);
  Serial.print(" GY:");
  Serial.println(movement.getYaw(), 3);
}
