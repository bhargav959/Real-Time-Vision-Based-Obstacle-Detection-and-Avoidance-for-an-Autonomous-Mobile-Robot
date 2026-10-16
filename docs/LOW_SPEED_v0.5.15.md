# v0.5.15 — Extra-Low-Speed Crawl

## Synchronized speed values

| Setting | App | Arduino |
|---|---:|---:|
| Hard PWM cap | 40 | 40 |
| Minimum moving PWM floor | 25 | 25 |
| Slight-turn outer wheel minimum | 33 | 33 |
| Startup kick | 30 for 40 ms, only below requested PWM 28 | same |
| No-echo cap | 30 | 30 |
| Backoff cap | 30 | 30 |
| Active scan starts at | 26 | receives command |
| Active scan maximum, valid sonar | 34 | hard global cap 40 |
| Active scan maximum, NO_ECHO | 30 | no-echo clamp 30 |
| App live speed ramp-up | +5 PWM per 100 ms tick | n/a |
| Scan keepalive | every 100 ms | 250 ms dead-man lease |

## Rationale

This lowers the prior release's 60 PWM hard cap to 40, starts active scanning at 26 rather than 35, caps scan escalation at 34 with fresh sonar, reduces no-echo motion and reverse to 30, and halves the live ramp increment from 12 to 5. The virtual drive model reads the same `MotorCalibration` values, and the current `REALISTIC` model deadband placeholder is 24, just under the firmware's 25 PWM moving floor.

These are conservative starting values, not measured wheel speeds. If PWM 26 does not rotate the chassis, the controller can only step to 31 then at most 34 (30 on NO_ECHO); it will stop/abort if it cannot verify a turn. Measure the minimum reliable loaded PWM before raising limits. Lower PWM can cause stalling, asymmetric turning or no motion rather than reliably making the robot slower.

## Files that must stay synchronized

- `app/src/main/java/com/example/robotcar/navigation/Models.kt`
- `app/src/main/java/com/example/robotcar/navigation/SafetyConstants.kt`
- `app/src/main/java/com/example/robotcar/navigation/ActivePathDiscovery.kt`
- `app/src/main/java/com/example/robotcar/navigation/LiveDecisionStabilizer.kt`
- `app/src/main/java/com/example/robotcar/video/VirtualScenarioProvider.kt` (should derive constants, not duplicate them)
- `app/src/main/java/com/example/robotcar/video/SimRealism.kt` (deadband placeholder)
- `docs/arduino/RobotCarObstacleNavigation.ino`
