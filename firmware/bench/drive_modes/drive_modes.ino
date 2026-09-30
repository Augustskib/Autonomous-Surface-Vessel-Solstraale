/*
 * drive_modes.ino
 *
 * Runs one motor three ways to compare them on the bench:
 *   analogWrite    hardware PWM, as boat_motor_controller.ino does
 *   software PWM   ~100 Hz bit-banged, slow enough for any H-bridge
 *   full DC        no switching, full pack voltage (control case)
 * If software PWM or DC turns the motor but analogWrite does not, the PWM
 * frequency is the problem.
 *
 * Every run stops itself and drops the enables. Props off, motors clamped.
 * Keep DC runs short: the motors are rated below full pack voltage.
 *
 * Commands:  1-3 left (analog, soft, DC)   4-6 right   0 stop   ? help
 */

// Same pin map as boat_motor_controller.ino.
const uint8_t L_RPWM = 5,  L_LPWM = 6,  L_R_EN = 2, L_L_EN = 4;
const uint8_t R_RPWM = 9,  R_LPWM = 10, R_R_EN = 7, R_L_EN = 8;

const unsigned long RUN_MS    = 3000;   // PWM modes
const unsigned long DC_RUN_MS = 1500;   // full DC, kept short
const uint8_t PWM_DUTY = 128;           // 50% of 255

String inBuf = "";

void allOff() {
  uint8_t pins[] = {L_RPWM, L_LPWM, R_RPWM, R_LPWM};
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
  uint8_t ens[] = {L_R_EN, L_L_EN, R_R_EN, R_L_EN};
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(ens[i], OUTPUT);
    digitalWrite(ens[i], LOW);
  }
}

void enable(uint8_t rEn, uint8_t lEn) {
  digitalWrite(rEn, HIGH);
  digitalWrite(lEn, HIGH);
}

void runAnalog(uint8_t rpwm, uint8_t lpwm, uint8_t rEn, uint8_t lEn) {
  Serial.println(F("  analogWrite 50% for 3s..."));
  analogWrite(lpwm, 0);
  enable(rEn, lEn);
  analogWrite(rpwm, PWM_DUTY);
  delay(RUN_MS);
  allOff();
  Serial.println(F("  stopped."));
}

void runSoftPwm(uint8_t rpwm, uint8_t lpwm, uint8_t rEn, uint8_t lEn) {
  Serial.println(F("  software PWM ~100Hz 50% for 3s..."));
  digitalWrite(lpwm, LOW);
  enable(rEn, lEn);
  unsigned long end = millis() + RUN_MS;
  while (millis() < end) {
    digitalWrite(rpwm, HIGH);
    delay(5);                // 5 ms on
    digitalWrite(rpwm, LOW);
    delay(5);                // 5 ms off: 100 Hz, 50%
  }
  allOff();
  Serial.println(F("  stopped."));
}

void runDC(uint8_t rpwm, uint8_t lpwm, uint8_t rEn, uint8_t lEn) {
  Serial.println(F("  FULL DC for 1.5s - full pack voltage..."));
  digitalWrite(lpwm, LOW);
  enable(rEn, lEn);
  digitalWrite(rpwm, HIGH);
  delay(DC_RUN_MS);
  allOff();
  Serial.println(F("  stopped. FEEL THE MOTOR BODY."));
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== drive_modes: analogWrite vs soft PWM vs DC ==="));
  Serial.println(F("PROPS OFF. Each run stops itself. No run stays on."));
  Serial.println();
  Serial.println(F("  1  LEFT   analogWrite 50%      (what the firmware does)"));
  Serial.println(F("  2  LEFT   software PWM 100Hz   (slow, any bridge follows)"));
  Serial.println(F("  3  LEFT   full DC 1.5s         (control - no switching)"));
  Serial.println(F("  4  RIGHT  analogWrite 50%"));
  Serial.println(F("  5  RIGHT  software PWM 100Hz"));
  Serial.println(F("  6  RIGHT  full DC 1.5s"));
  Serial.println(F("  0  all stop      ?  this list"));
  Serial.println();
  Serial.println(F("Run 1, then 2, then 3. Note which ones move the motor."));
  Serial.println();
}

void setup() {
  allOff();
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { ; }
  printHelp();
  Serial.println(F("Ready. Everything off."));
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      inBuf.trim();
      if (inBuf.length() > 0) {
        if      (inBuf == "?") printHelp();
        else if (inBuf == "0") { allOff(); Serial.println(F("  all off.")); }
        else if (inBuf == "1") runAnalog (L_RPWM, L_LPWM, L_R_EN, L_L_EN);
        else if (inBuf == "2") runSoftPwm(L_RPWM, L_LPWM, L_R_EN, L_L_EN);
        else if (inBuf == "3") runDC     (L_RPWM, L_LPWM, L_R_EN, L_L_EN);
        else if (inBuf == "4") runAnalog (R_RPWM, R_LPWM, R_R_EN, R_L_EN);
        else if (inBuf == "5") runSoftPwm(R_RPWM, R_LPWM, R_R_EN, R_L_EN);
        else if (inBuf == "6") runDC     (R_RPWM, R_LPWM, R_R_EN, R_L_EN);
        else Serial.println(F("  unknown. ? for the list."));
      }
      inBuf = "";
    } else if (inBuf.length() < 8) {
      inBuf += c;
    }
  }
}
