/*
 * pin_walk.ino
 *
 * Harness check. Drives one Arduino pin HIGH at a time so you can find which
 * BTS7960 logic pin it is wired to. Does not run the motors on purpose.
 *
 * Disconnect the LiPo first. With battery power on the drivers, driving an
 * enable and a PWM pin HIGH runs a motor at full throttle.
 *
 * Use:
 *   1. Upload, open a serial monitor at 115200.
 *   2. Multimeter on DC volts, black probe on Arduino GND.
 *   3. Type a pin number. Probe the 8 logic pins on the drivers; one reads ~5 V.
 *
 * Expected map (boat_motor_controller.ino):
 *   Left   RPWM D5  LPWM D6   R_EN D2  L_EN D4
 *   Right  RPWM D9  LPWM D10  R_EN D7  L_EN D8
 * If the harness disagrees, rewire to match the firmware.
 *
 * Commands:  <number> drive that pin HIGH   0 all LOW   walk auto-cycle 4 s each   ? help
 */

// Wider than the firmware map so miswired pins still show up.
const uint8_t CANDIDATES[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const uint8_t N_CANDIDATES = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

const long SERIAL_BAUD = 115200;

String inBuf = "";
int heldPin = -1;

void allLow() {
  for (uint8_t i = 0; i < N_CANDIDATES; i++) {
    pinMode(CANDIDATES[i], OUTPUT);
    digitalWrite(CANDIDATES[i], LOW);
  }
  heldPin = -1;
}

bool isCandidate(int pin) {
  for (uint8_t i = 0; i < N_CANDIDATES; i++) {
    if (CANDIDATES[i] == pin) return true;
  }
  return false;
}

void drive(int pin) {
  allLow();
  if (pin < 0) {
    Serial.println(F("ALL PINS LOW - safe state"));
    return;
  }
  if (!isCandidate(pin)) {
    Serial.print(F("D"));
    Serial.print(pin);
    Serial.println(F(" is not in the candidate list. Type ? for the list."));
    return;
  }
  digitalWrite(pin, HIGH);
  heldPin = pin;
  Serial.print(F("D"));
  Serial.print(pin);
  Serial.println(F(" is now HIGH (~5V). Probe the BTS7960 logic pins."));
  Serial.println(F("  Exactly one should read ~5V. Note which. Then pick the next pin."));
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== pin_walk: harness verification ==="));
  Serial.println(F("LiPo MUST be disconnected. This drives enables and PWM pins HIGH."));
  Serial.println();
  Serial.println(F("Black multimeter probe on Arduino GND, red probe on each BTS7960 logic pin."));
  Serial.println();
  Serial.print(F("Candidate pins: "));
  for (uint8_t i = 0; i < N_CANDIDATES; i++) {
    Serial.print(F("D"));
    Serial.print(CANDIDATES[i]);
    if (i < N_CANDIDATES - 1) Serial.print(F(", "));
  }
  Serial.println();
  Serial.println();
  Serial.println(F("Commands:  <number> = drive that pin HIGH"));
  Serial.println(F("           0        = all LOW (safe)"));
  Serial.println(F("           walk     = auto-cycle, 4s per pin"));
  Serial.println(F("           ?        = this help"));
  Serial.println();
  Serial.println(F("What the firmware expects, for comparison:"));
  Serial.println(F("  LEFT  RPWM=D5  LPWM=D6   R_EN=D2  L_EN=D4"));
  Serial.println(F("  RIGHT RPWM=D9  LPWM=D10  R_EN=D7  L_EN=D8"));
  Serial.println();
}

void autoWalk() {
  Serial.println(F("Auto-walk: 4 seconds per pin. Send any character to stop."));
  for (uint8_t i = 0; i < N_CANDIDATES; i++) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      break;
    }
    allLow();
    digitalWrite(CANDIDATES[i], HIGH);
    Serial.print(F("  D"));
    Serial.print(CANDIDATES[i]);
    Serial.println(F(" HIGH"));
    unsigned long start = millis();
    while (millis() - start < 4000) {
      if (Serial.available()) break;
      delay(20);
    }
  }
  allLow();
  Serial.println(F("Auto-walk done. All pins LOW."));
}

void setup() {
  allLow();

  Serial.begin(SERIAL_BAUD);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { ; }

  printHelp();
  Serial.println(F("Ready. All pins LOW."));
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      inBuf.trim();
      if (inBuf.length() > 0) {
        if (inBuf == "?") {
          printHelp();
        } else if (inBuf == "walk") {
          autoWalk();
        } else if (inBuf == "0") {
          drive(-1);
        } else {
          drive(inBuf.toInt());
        }
      }
      inBuf = "";
    } else if (inBuf.length() < 16) {
      inBuf += c;
    }
  }
}
