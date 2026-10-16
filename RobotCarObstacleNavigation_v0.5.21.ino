#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// ROBOT CAR - ARDUINO UNO + L298N (camera-only default; optional HC-SR04)
// Matching command protocol for RobotCar Android App v0.5.21 (turn factor 1.45, straight cap 32 PWM)
//
// L298N:  ENA=D9 (PWM, jumper removed)  IN1=D2  IN2=D3
//         ENB=D10 (PWM, jumper removed) IN3=D4  IN4=D5
// Optional HC-SR04: TRIG=D6 ECHO=D7. Disabled by default because no sensor is visible in the supplied chassis photo.
//
// Android protocol @ 115200 8-N-1, every line newline terminated:
//   F,32  SL,48  SR,48  L,58  R,58  SCAN_L,37  SCAN_R,37  S,0
//   BACKOFF,25 is a one-shot blind reverse pulse (250 ms max); continuous REV stays disabled.
//   DISTANCE   -> DISTANCE=42.5 | DISTANCE=NO_ECHO | DISTANCE=TOO_SOON
//   Every accepted forward/turn ack carries the sonar reading that authorised it:
//   RESULT=F,32,D=41.2 | RESULT=SL,30,D=NO_ECHO   (gives the app fresh sonar feedback)
//   STATUS     -> STATUS=OK
// Boot: prints ROBOT_READY (opening the USB port resets an UNO; the app waits for this/STATUS).
// Other output: RESULT=<cmd>,<pwm> | RESULT=STOP_OBSTACLE:<cm> | RESULT=STOP_TURN_OBSTACLE:<cm>
//               RESULT=BACKOFF_STARTED,<pwm>,250 | RESULT=BACKOFF_COMPLETE (bench override only)
//               RESULT=SCAN_PIVOT_STARTED,<L/R>,<pwm>,LEASE=250 | RESULT=SCAN_PIVOT_REFRESHED,<L/R>,<pwm>,D=<cm>
//               SAFETY_STOP=WATCHDOG | ERROR=...
//
// Hardware notes (see docs/SIM_REALITY_CALIBRATION_v0.5.11.md):
//  * Fit 10k pull-DOWN resistors from ENA, ENB (and IN1..IN4) to GND: with the L298N jumpers removed
//    those pins float while the UNO resets/boots and can spin the motors.
//  * Fit 100 nF ceramic caps across each motor's terminals and give the motors their own battery
//    (common GND). Motor start-up spikes can brown-out the UNO if it is powered from the phone's USB.
//
// Safety ordering: the APP stops at 30 cm (SafetyConstants.ULTRASONIC_STOP_CM) and uses fresh
// camera/sonar clearance before pivoting. The firmware is the BACKSTOP (28 cm forward / 20 cm pivot).
// A single short blind BACKOFF is enabled by request to create turning room; rear clearance is not
// sensed, so use only in a supervised, physically clear test area. Unbounded REV remains disabled.
// Keep these numbers in sync with SafetyConstants.kt.
// ============================================================

const byte ENA = 9;
const byte IN1 = 2;
const byte IN2 = 3;

const byte ENB = 10;
const byte IN3 = 4;
const byte IN4 = 5;

const byte TRIG_PIN = 6;
const byte ECHO_PIN = 7;
// Set true ONLY after installing and wiring an HC-SR04 on D6/D7. Camera-only mode relies on Android
// fresh-frame safety checks and deliberately has no blind reverse.
const bool USE_HC_SR04 = false;

// ---- Safety thresholds (keep in sync with SafetyConstants.kt) ----
const float FORWARD_STOP_DISTANCE_CM = 28.0;
const float TURN_STOP_DISTANCE_CM = 20.0;
const byte  NO_ECHO_MAX_PWM = 30;          // NO_ECHO gives no close-range assurance: cap speed

// Autonomous in-motion sonar guard (v0.5.11): the firmware used to look at the sonar only when a
// motion command arrived, so an app stall could leave the robot driving for the full 400 ms
// watchdog (travel depends on calibrated wheel speed; normal forward output is capped at PWM 32). While a translating/pivot command is active, ping every
// GUARD_INTERVAL_MS and stop on two consecutive close readings (or one very close one).
const unsigned long GUARD_INTERVAL_MS = 60UL;
const float GUARD_IMMEDIATE_FRACTION = 0.7;  // one reading below limit*0.7 stops at once
// HC-SR04 needs roughly 60 ms between pings. Do not burst multiple pings 10 ms apart: echoes can
// overlap and produce dangerously optimistic readings. All actual triggers are rate-limited below.

// ---- Motor behaviour (CALIBRATE on the real chassis) ----
const byte ARDUINO_MAX_PWM = 58;          // hard cap permits the +45% turn profile
const byte STRAIGHT_MAX_PWM = 32;        // F/REV are capped at 80% of the 40 PWM base ceiling
const byte MIN_MOTOR_PWM = 25;             // minimum moving PWM; calibrate under chassis load
const byte SLIGHT_TURN_OUTER_MIN_PWM = 48;  // sharper differential steering
const byte KICK_PWM = 30;                  // brief startup boost for low requested movement speeds
const unsigned long KICK_MS = 40UL;
const byte KICK_ONLY_BELOW_PWM = 28;       // only boost very low requested PWM values

// One-shot blind reverse support. This is time- and PWM-bounded, but cannot prove rear clearance.
const byte BACKOFF_MAX_PWM = 25;
const unsigned long BACKOFF_DURATION_MS = 250UL;
const unsigned long SCAN_COMMAND_LEASE_MS = 250UL; // loss of Android keepalive stops the scan pivot

// HC-SR04 max range ~4 m = ~23.5 ms round trip.
const unsigned long ECHO_TIMEOUT_US = 24000UL;
const unsigned long MIN_PING_INTERVAL_MS = 60UL;
const unsigned long COMMAND_TIMEOUT_MS = 400UL;   // app sends every 100 ms
// Only the bounded BACKOFF pulse is enabled. Do not enable continuous REV without rear sensing.
const bool ALLOW_BLIND_BACKOFF = true;
const bool ALLOW_UNBOUNDED_REVERSE = false;

// Flip if the physical motor wiring is opposite to the logical LEFT/RIGHT convention.
const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;

const byte CMD_MAX = 24;
char cmdBuf[CMD_MAX + 1];
byte cmdLen = 0;
bool cmdOverflow = false;

unsigned long lastMotionCommandTime = 0UL;
bool motorsRunning = false;
byte leftTarget = 0;
byte rightTarget = 0;
unsigned long kickUntil = 0UL;    // 0 = no kick in progress
bool gNoEcho = false;             // set by the last safety check
bool timedBackoffActive = false;
bool backoffCommandLatched = false;
unsigned long backoffStartedAt = 0UL;
float gLastDistanceCm = -1.0;     // last sonar reading (<=0 = NO_ECHO)
unsigned long lastSonarPingAt = 0UL;
bool haveSonarPing = false;
float guardLimitCm = 0.0;         // 0 = in-motion guard disabled
byte guardHits = 0;
unsigned long lastGuardPingAt = 0UL;
bool scanPivotActive = false;
bool scanPivotLatched = false;
unsigned long scanCommandLastAt = 0UL;
bool activePivotLeft = false;

// Forward declarations (the Arduino IDE adds these automatically; explicit for other toolchains).
void readCommands();
void processCommand(char* line);
void reportResult(const char* action, int speed);
void runMotionGuard(unsigned long now);
bool safeForForward();
bool safeForTurn();
bool safeForScanTurn();
bool safeForReverse();
byte floorPwm(int pwm);
void applyDrive(int l, int r, bool allowStartupKick = true);
void forward(byte speed);
void reverseMotor(byte speed);
void slightLeft(byte speed);
void slightRight(byte speed);
void left(byte speed);
void right(byte speed);
void setMotorDirection(bool leftForward, bool rightForward);
void stopMotors();
void startTimedBackoff(byte speed);
void startScanPivot(byte speed, bool pivotLeft);
void reportDistance();
float getDistanceCm();
bool isNumber(const char* s);
byte limitedSpeed(int speed);

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  stopMotors();
  lastMotionCommandTime = millis();

  Serial.println("ROBOT_READY");
}

void loop() {
  readCommands();

  const unsigned long now = millis();

  if (kickUntil != 0UL && (long)(now - kickUntil) >= 0) {
    kickUntil = 0UL;
    analogWrite(ENA, leftTarget);
    analogWrite(ENB, rightTarget);
  }

  if (timedBackoffActive && (now - backoffStartedAt >= BACKOFF_DURATION_MS)) {
    timedBackoffActive = false;
    stopMotors();
    Serial.println("RESULT=BACKOFF_COMPLETE");
  }
  if (scanPivotActive && (now - scanCommandLastAt >= SCAN_COMMAND_LEASE_MS)) {
    // Dead-man lease: scan rotation is continuous only while Android refreshes the same
    // direction each control tick. If camera processing, USB, or the app stalls, stop.
    scanPivotActive = false;
    scanPivotLatched = false;
    stopMotors();
    Serial.println("SAFETY_STOP=SCAN_COMMAND_TIMEOUT");
  }

  runMotionGuard(now);

  if (motorsRunning && (now - lastMotionCommandTime > COMMAND_TIMEOUT_MS)) {
    stopMotors();
    Serial.println("SAFETY_STOP=WATCHDOG");
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdOverflow) {
        stopMotors();
        Serial.println("ERROR=LINE_TOO_LONG");
      } else if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        processCommand(cmdBuf);
      }
      cmdLen = 0;
      cmdOverflow = false;
    } else if (cmdLen < CMD_MAX) {
      cmdBuf[cmdLen++] = (c >= 'a' && c <= 'z') ? (char)(c - 32) : c;
    } else {
      cmdOverflow = true;
    }
  }
}

bool isNumber(const char* s) {
  if (*s == '\0') return false;
  for (; *s; s++) {
    if (*s < '0' || *s > '9') return false;
  }
  return true;
}

byte limitedSpeed(int speed) {
  if (speed < 0) speed = 0;
  if (speed > ARDUINO_MAX_PWM) speed = ARDUINO_MAX_PWM;
  if (gNoEcho && speed > NO_ECHO_MAX_PWM) speed = NO_ECHO_MAX_PWM;
  return (byte)speed;
}

void processCommand(char* line) {
  char* comma = strchr(line, ',');
  int speed = 0;
  bool haveSpeed = false;
  if (comma != NULL) {
    *comma = '\0';
    if (isNumber(comma + 1)) {
      speed = atoi(comma + 1);
      haveSpeed = true;
    } else {
      stopMotors();
      Serial.println("ERROR=BAD_SPEED");
      return;
    }
  }
  const char* action = line;

  if (strcmp(action, "S") == 0) {
    timedBackoffActive = false;
    backoffCommandLatched = false;
    scanPivotActive = false;
    scanPivotLatched = false;
    stopMotors();
    Serial.println("RESULT=S,0");
    return;
  }
  if (strcmp(action, "DISTANCE") == 0) {
    reportDistance();
    return;
  }
  if (strcmp(action, "STATUS") == 0) {
    Serial.println("STATUS=OK");
    return;
  }

  const bool isMotion = strcmp(action, "F") == 0 || strcmp(action, "SL") == 0 ||
                        strcmp(action, "SR") == 0 || strcmp(action, "L") == 0 ||
                        strcmp(action, "R") == 0 || strcmp(action, "REV") == 0 ||
                        strcmp(action, "BACKOFF") == 0 || strcmp(action, "SCAN_L") == 0 ||
                        strcmp(action, "SCAN_R") == 0;
  if (!isMotion) {
    stopMotors();
    Serial.print("ERROR=UNKNOWN_COMMAND:");
    Serial.println(action);
    return;
  }
  if (!haveSpeed) {   // never guess a speed for a motion command
    stopMotors();
    Serial.println("ERROR=MISSING_SPEED");
    return;
  }

  if (strcmp(action, "BACKOFF") == 0) {
    // The front sensor cannot prove the reverse path clear. This command is enabled only as a
    // single firmware-timed pulse; repeated app keepalives cannot extend/restart the pulse.
    if (!ALLOW_BLIND_BACKOFF) {
      stopMotors();
      Serial.println("ERROR=BACKOFF_DISABLED_NO_REAR_SENSOR");
      return;
    }
    // BACKOFF is idempotent for the duration of the app's BACKOFF phase: repeated 10 Hz
    // commands must not restart the timer or turn the pulse into continuous reverse motion.
    if (timedBackoffActive || backoffCommandLatched) return;
    if (scanPivotActive) { scanPivotActive = false; scanPivotLatched = false; stopMotors(); }
    byte s = limitedSpeed(speed);
    if (s > BACKOFF_MAX_PWM) s = BACKOFF_MAX_PWM;
    backoffCommandLatched = true;
    startTimedBackoff(s);
    return;
  }

  if (strcmp(action, "SCAN_L") == 0 || strcmp(action, "SCAN_R") == 0) {
    const bool requestLeft = strcmp(action, "SCAN_L") == 0;

    // While scanning, the app refreshes this command each 100 ms control tick. Repeated
    // same-direction commands keep the motors running (no stop between scan steps), verify
    // one fresh ultrasonic sample, and update PWM without the normal startup kick.
    if (scanPivotActive) {
      if (requestLeft != activePivotLeft) {
        // Direction reversal is only allowed after the app has explicitly stopped and sampled.
        scanPivotActive = false;
        scanPivotLatched = false;
        stopMotors();
        Serial.println("ERROR=SCAN_DIRECTION_CHANGE_REQUIRES_STOP");
        return;
      }
      if (!safeForScanTurn()) {
        scanPivotActive = false;
        scanPivotLatched = false;
        return;
      }
      const byte s = limitedSpeed(speed);
      lastMotionCommandTime = millis();
      scanCommandLastAt = lastMotionCommandTime;
      guardLimitCm = 0.0; // safeForScanTurn supplies one fresh sonar check per keepalive
      applyDrive(s, s, false);
      Serial.print("RESULT=SCAN_PIVOT_REFRESHED,");
      Serial.print(requestLeft ? "L," : "R,");
      Serial.print(s);
      Serial.print(",D=");
      if (gLastDistanceCm > 0.0) Serial.println(gLastDistanceCm, 1);
      else Serial.println("NO_ECHO");
      return;
    }
    if (scanPivotLatched) return;
    if (timedBackoffActive) { timedBackoffActive = false; backoffCommandLatched = false; stopMotors(); }
    if (!safeForTurn()) return;
    scanPivotLatched = true;
    startScanPivot(limitedSpeed(speed), requestLeft);
    return;
  }

  // Any other movement command cancels the active scan/backoff, then takes over normally.
  if (timedBackoffActive || scanPivotActive) {
    timedBackoffActive = false;
    scanPivotActive = false;
    stopMotors();
  }
  backoffCommandLatched = false;
  scanPivotLatched = false;

  if (strcmp(action, "F") == 0 || strcmp(action, "SL") == 0 || strcmp(action, "SR") == 0) {
    if (!safeForForward()) return;
    byte s = limitedSpeed(speed);
    if (strcmp(action, "F") == 0 && s > STRAIGHT_MAX_PWM) s = STRAIGHT_MAX_PWM;
    // Only an accepted movement command refreshes the watchdog.
    lastMotionCommandTime = millis();
    if (strcmp(action, "F") == 0) forward(s);
    else if (strcmp(action, "SL") == 0) slightLeft(s);
    else slightRight(s);
    guardLimitCm = FORWARD_STOP_DISTANCE_CM; guardHits = 0; lastGuardPingAt = millis();
    reportResult(action, s);
  } else if (strcmp(action, "L") == 0 || strcmp(action, "R") == 0) {
    if (!safeForTurn()) return;
    const byte s = limitedSpeed(speed);
    // Telemetry and rejected turns cannot keep a previous drive command alive.
    lastMotionCommandTime = millis();
    if (strcmp(action, "L") == 0) left(s); else right(s);
    guardLimitCm = TURN_STOP_DISTANCE_CM; guardHits = 0; lastGuardPingAt = millis();
    reportResult(action, s);
  } else {  // REV
    // Continuous REV remains disabled without rear sensing; a front sonar reading is not a proxy
    // for what is behind the chassis. Only timed BACKOFF is available for recovery.
    if (!ALLOW_UNBOUNDED_REVERSE) {
      stopMotors();
      Serial.println("ERROR=REVERSE_DISABLED_NO_REAR_SENSOR");
      return;
    }
    if (!safeForReverse()) return;
    byte s = limitedSpeed(speed);
    if (s > STRAIGHT_MAX_PWM) s = STRAIGHT_MAX_PWM;
    lastMotionCommandTime = millis();
    reverseMotor(s);
    reportResult("REV", s);
  }
}

void reportResult(const char* action, int speed) {
  Serial.print("RESULT=");
  Serial.print(action);
  Serial.print(",");
  Serial.print(speed);
  // Piggyback the reading that authorised this command so the app's sonar is as fresh as its commands.
  Serial.print(",D=");
  if (gLastDistanceCm > 0.0) Serial.println(gLastDistanceCm, 1);
  else Serial.println("NO_ECHO");
}

// Independent of the app: while moving, keep checking the sonar between commands.
void runMotionGuard(unsigned long now) {
  if (!USE_HC_SR04 || !motorsRunning || guardLimitCm <= 0.0 || scanPivotActive || timedBackoffActive) return; // scan pivot checks sonar on each keepalive
  if (now - lastGuardPingAt < GUARD_INTERVAL_MS) return;
  lastGuardPingAt = now;
  const float d = getDistanceCm();   // single ping (<= 24 ms); a miss is NOT evidence of an obstacle
  if (d <= 0.0) { guardHits = 0; return; }
  const bool veryClose = d < guardLimitCm * GUARD_IMMEDIATE_FRACTION;
  if (d < guardLimitCm) guardHits++; else guardHits = 0;
  if (veryClose || guardHits >= 2) {
    stopMotors();
    Serial.print(guardLimitCm == TURN_STOP_DISTANCE_CM ? "RESULT=STOP_TURN_OBSTACLE:" : "RESULT=STOP_OBSTACLE:");
    Serial.println(d, 1);
  }
}

// Forward-type motion. A close echo stops. NO_ECHO is NOT treated as clear: it is allowed only at a
// capped speed (the app additionally requires camera-verified free space before sending anything).
bool safeForForward() {
  if (!USE_HC_SR04) { gNoEcho = false; gLastDistanceCm = -1.0; return true; }
  // One ping per command; global rate limiting prevents HC-SR04 echo cross-talk.
  const float d = getDistanceCm();
  gNoEcho = (d <= 0.0);
  gLastDistanceCm = d;

  if (d > 0.0 && d < FORWARD_STOP_DISTANCE_CM) {
    stopMotors();
    Serial.print("RESULT=STOP_OBSTACLE:");
    Serial.println(d, 1);
    return false;
  }
  return true;
}

bool safeForScanTurn() {
  if (!USE_HC_SR04) { gNoEcho = false; gLastDistanceCm = -1.0; return true; }
  // Single ping per 100 ms keepalive avoids multi-ping blocking between scan steps.
  // NO_ECHO is not interpreted as clear: limitedSpeed() clamps motion to NO_ECHO_MAX_PWM,
  // while the Android side still requires fresh camera-based clearance.
  const float d = getDistanceCm();
  gNoEcho = (d <= 0.0);
  gLastDistanceCm = d;
  if (d > 0.0 && d < TURN_STOP_DISTANCE_CM) {
    stopMotors();
    Serial.print("RESULT=STOP_TURN_OBSTACLE:");
    Serial.println(d, 1);
    return false;
  }
  return true;
}

bool safeForTurn() {
  if (!USE_HC_SR04) { gNoEcho = false; gLastDistanceCm = -1.0; return true; }
  const float d = getDistanceCm();
  gNoEcho = (d <= 0.0);
  gLastDistanceCm = d;

  if (d > 0.0 && d < TURN_STOP_DISTANCE_CM) {
    stopMotors();
    Serial.print("RESULT=STOP_TURN_OBSTACLE:");
    Serial.println(d, 1);
    return false;
  }
  return true;
}

bool safeForReverse() {
  // Only relevant if continuous REV is enabled; continuous REV is disabled in this build.
  // The front-facing sonar must never be used to infer rear clearance.
  gNoEcho = true;
  gLastDistanceCm = getDistanceCm(); // telemetry only; not a reverse-safety measurement
  return true;
}

// ---- Motor output -------------------------------------------------------

byte floorPwm(int pwm) {
  if (pwm <= 0) return 0;
  return (byte)(pwm < MIN_MOTOR_PWM ? MIN_MOTOR_PWM : pwm);
}

void applyDrive(int l, int r, bool allowStartupKick) {
  const byte lt = floorPwm(l);
  const byte rt = floorPwm(r);
  const bool wasStopped = !motorsRunning;
  motorsRunning = (lt > 0 || rt > 0);
  leftTarget = lt;
  rightTarget = rt;

  const byte top = lt > rt ? lt : rt;
  if (allowStartupKick && motorsRunning && wasStopped && top < KICK_ONLY_BELOW_PWM) {
    kickUntil = millis() + KICK_MS;
    if (kickUntil == 0UL) kickUntil = 1UL;
    analogWrite(ENA, lt > 0 ? KICK_PWM : 0);
    analogWrite(ENB, rt > 0 ? KICK_PWM : 0);
  } else if (kickUntil == 0UL) {
    analogWrite(ENA, lt);
    analogWrite(ENB, rt);
  }  // else: kick in progress; loop() applies the targets when it ends
}

void forward(byte speed) {
  setMotorDirection(true, true);
  applyDrive(speed, speed);
}

void reverseMotor(byte speed) {
  setMotorDirection(false, false);
  applyDrive(speed, speed);
}

byte slightTurnOuterSpeed(byte speed) {
  byte outerFloor = gNoEcho ? MIN_MOTOR_PWM : SLIGHT_TURN_OUTER_MIN_PWM;
  byte outerCap = gNoEcho ? NO_ECHO_MAX_PWM : ARDUINO_MAX_PWM;
  byte outer = speed < outerFloor ? outerFloor : speed;
  if (outer > outerCap) outer = outerCap;
  return outer;
}

void slightLeft(byte speed) {
  if (speed == 0) { stopMotors(); return; }
  setMotorDirection(true, true);
  // Normally the outer wheel has a 40 PWM floor to preserve differential steering. When sonar
  // reports NO_ECHO, the 30 PWM safety cap takes priority; the inner wheel still floors at 25.
  const byte outer = slightTurnOuterSpeed(speed);
  applyDrive((int)outer * 45 / 100, outer);
}

void slightRight(byte speed) {
  if (speed == 0) { stopMotors(); return; }
  setMotorDirection(true, true);
  const byte outer = slightTurnOuterSpeed(speed);
  applyDrive(outer, (int)outer * 45 / 100);
}

void left(byte speed) {
  setMotorDirection(false, true);
  applyDrive(speed, speed);
}

void right(byte speed) {
  setMotorDirection(true, false);
  applyDrive(speed, speed);
}

void setMotorDirection(bool leftForward, bool rightForward) {
  if (INVERT_LEFT_MOTOR) leftForward = !leftForward;
  if (INVERT_RIGHT_MOTOR) rightForward = !rightForward;

  digitalWrite(IN1, leftForward ? HIGH : LOW);
  digitalWrite(IN2, leftForward ? LOW : HIGH);

  digitalWrite(IN3, rightForward ? HIGH : LOW);
  digitalWrite(IN4, rightForward ? LOW : HIGH);
}

void stopMotors() {
  motorsRunning = false;
  // Any direct stop/error path invalidates a previously active scan lease. The scan handler
  // explicitly re-arms only after a new, clearance-checked SCAN_L/SCAN_R command.
  scanPivotActive = false;
  scanPivotLatched = false;
  guardLimitCm = 0.0;
  guardHits = 0;
  kickUntil = 0UL;
  leftTarget = 0;
  rightTarget = 0;
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void startTimedBackoff(byte speed) {
  // The forward-facing HC-SR04 cannot verify rear clearance; this is intentionally short and slow.
  // Do not call safeForReverse() here because it measures the obstacle in FRONT of the robot.
  if (speed == 0) {
    stopMotors();
    Serial.println("RESULT=BACKOFF_SKIPPED_ZERO_SPEED");
    return;
  }
  gNoEcho = true;
  timedBackoffActive = true;
  backoffStartedAt = millis();
  lastMotionCommandTime = backoffStartedAt;
  reverseMotor(speed);
  Serial.print("RESULT=BACKOFF_STARTED,");
  Serial.print(speed);
  Serial.print(",");
  Serial.println(BACKOFF_DURATION_MS);
}

void startScanPivot(byte speed, bool pivotLeft) {
  if (speed == 0) {
    stopMotors();
    Serial.println("RESULT=SCAN_PIVOT_SKIPPED_ZERO_SPEED");
    return;
  }
  scanPivotActive = true;
  activePivotLeft = pivotLeft;
  scanCommandLastAt = millis();
  lastMotionCommandTime = scanCommandLastAt;
  setMotorDirection(!pivotLeft, pivotLeft);
  // Scan pivots deliberately bypass the normal startup kick to keep active scanning slow.
  // Rotation continues while same-direction commands refresh the 250 ms dead-man lease.
  applyDrive(speed, speed, false);
  Serial.print("RESULT=SCAN_PIVOT_STARTED,");
  Serial.print(pivotLeft ? "L," : "R,");
  Serial.print(speed);
  Serial.print(",LEASE=");
  Serial.println(SCAN_COMMAND_LEASE_MS);
}

// ---- Ultrasonic ---------------------------------------------------------

void reportDistance() {
  // Telemetry is advisory. Do not issue a second ping inside the 60 ms quiet interval;
  // the next motion acknowledgement supplies a fresh reading. TOO_SOON is ignored by the app.
  if (haveSonarPing && millis() - lastSonarPingAt < MIN_PING_INTERVAL_MS) {
    Serial.println("DISTANCE=TOO_SOON");
    return;
  }
  const float d = getDistanceCm();
  Serial.print("DISTANCE=");
  if (d <= 0.0) Serial.println("NO_ECHO");
  else Serial.println(d, 1);
}

// One ping at a time, respecting the HC-SR04 quiet interval across every caller.
float getDistanceCm() {
  if (!USE_HC_SR04) return -1.0;
  while (haveSonarPing && millis() - lastSonarPingAt < MIN_PING_INTERVAL_MS) {
    delay(1);
  }
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  lastSonarPingAt = millis();
  haveSonarPing = true;

  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0UL) return -1.0;
  return duration * 0.0343 / 2.0;
}
