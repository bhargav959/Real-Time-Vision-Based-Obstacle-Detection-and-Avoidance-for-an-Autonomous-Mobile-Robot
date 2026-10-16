# Android app speed values — v0.5.15

If updating the Android app manually rather than using the complete ZIP, update these files. The complete ZIP already contains these changes.

## 1. `app/src/main/java/com/example/robotcar/navigation/Models.kt`

Set `MotorCalibration` to:

```kotlin
const val ARDUINO_MAX_PWM = 40
const val MIN_MOTOR_PWM = 25
const val SLIGHT_TURN_OUTER_MIN_PWM = 33
const val STARTUP_KICK_PWM = 30
const val STARTUP_KICK_MS = 40L
const val STARTUP_KICK_THRESHOLD_PWM = 28
const val BACKOFF_PWM = 30
const val BACKOFF_DURATION_MS = 250L
const val CONTROL_TICK_MS = 100L
const val SCAN_START_PWM = 26
const val SCAN_MAX_PWM = 34
const val NOMINAL_SPEED_CM_S_AT_PWM_150 = 18f // keep only as uncalibrated placeholder
```

Do not change the nominal cm/s placeholder unless you measure actual loaded speed; the lower PWM limits already reduce commanded movement in the virtual motor model.

## 2. `app/src/main/java/com/example/robotcar/navigation/SafetyConstants.kt`

Set `NO_ECHO_MAX_PWM = 30`. `MIN_MOTOR_PWM` should remain an alias to `MotorCalibration.MIN_MOTOR_PWM`. Leave distance thresholds alone; they are safety clearances, not speed settings.

## 3. `app/src/main/java/com/example/robotcar/navigation/ActivePathDiscovery.kt`

Set the default `scanPwm` to `MotorCalibration.SCAN_START_PWM` (26), and ensure `maxScanPwm` is `MotorCalibration.SCAN_MAX_PWM` (34). Do not leave the old hard-coded 35.

## 4. `app/src/main/java/com/example/robotcar/navigation/LiveDecisionStabilizer.kt`

Set `pwmRampUpPerTick = 5` (previously 12). STOP and speed decreases must remain immediate.

## 5. `app/src/main/java/com/example/robotcar/video/VirtualScenarioProvider.kt`

It should continue to derive max PWM, minimum PWM, outer-turn PWM, startup kick and thresholds from `MotorCalibration`. Do not add independent duplicate numeric values.

## 6. `app/src/main/java/com/example/robotcar/video/SimRealism.kt`

The initial `REALISTIC.motorDeadbandPwm` is set to 24 to match the placeholder 25 PWM firmware floor. This is a placeholder only: update it after measuring the lowest PWM that moves both wheels under load.

## 7. `app/src/test/...`

Tests were updated to expect scan start 26 PWM, scan escalation up to 34 (30 with NO_ECHO), and the lower 5-PWM ramp. Keep those expectations aligned if you change the constants again.

## Firmware must match

Use `docs/arduino/RobotCarObstacleNavigation.ino` from this release. Do not pair the new app constants with v0.5.14 firmware, because the older sketch has a 60 PWM hard cap, 30 PWM motor floor, 40 PWM outer turn minimum, 50 PWM startup boost and 45 PWM NO_ECHO cap.

## Safety note

PWM is not a speed unit. A motor may not move reliably at 25–26 PWM under load. Test wheels lifted, then on the floor in a clear enclosed space with emergency STOP accessible. The one-shot reverse is slower and will likely travel a shorter distance than older versions; the HC-SR04 cannot see behind the robot.
