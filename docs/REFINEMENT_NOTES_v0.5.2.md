# v0.5.2 — architecture review refinements

Changes made against `RobotCar_Obstacle_Navigation_Architecture_v0.5.1`. **Verification status:** the
navigation package (planner, projector, tracker, speed and navigation controllers, virtual scenarios)
was compiled and unit-tested on a plain JVM — 32 tests pass. The Android parts (`MainActivity`,
`UsbSerialManager`) and the Arduino sketch were edited but NOT built here: open in Android Studio,
run `gradlew test`, and compile the sketch before use. Nothing was validated on the physical robot.

## Safety
| # | Change | Where |
|---|--------|-------|
| 1 | Ultrasonic `NO_ECHO` is no longer "clear". New `ultrasonicNoEcho`, `ultrasonicAgeMs`, `ultrasonicRequired` in the snapshot. Live mode: a reading older than 800 ms (or never received) → STOP; NO_ECHO → speed capped at 90 PWM. Virtual mode ignores the sensor. | `Models.kt`, `NavigationController.kt`, `MainActivity.kt` |
| 2 | Arduino no longer refuses to drive on NO_ECHO (open floor beyond ~4 m returns no echo). It takes a 3-ping median, treats 2 misses as NO_ECHO, and allows motion only at a capped PWM. **Trade-off:** NO_ECHO now relies on the camera for close-range assurance; check that surfaces in your test area reflect ultrasound. | `.ino` |
| 3 | One set of thresholds (`SafetyConstants.kt`, mirrored in the sketch). App stops at 30 cm and plans the stop; Arduino is the backstop at 28 cm forward / 20 cm pivot. | `SafetyConstants.kt`, `.ino` |
| 4 | `gapClearanceMarginCm` (live: 2 cm per side → 19 cm required). Virtual harness keeps margin 0 so the exact 15 cm spec threshold is still demonstrated. | `Models.kt`, `ObstaclePlanner.kt`, `MainActivity.kt` |
| 5 | App waits for `ROBOT_READY`/`STATUS=OK` after every connect (opening USB resets an UNO); polls `STATUS`; ARM disabled and AUTONOMOUS forced SAFE until ready. | `MainActivity.kt`, `UsbSerialManager.kt` |

## Architecture gaps
| # | Change |
|---|--------|
| 6 | New `ObstacleTracker`: obstacles accepted in 1 frame, held ~3 frames; FREE needs 2 consecutive free frames (otherwise UNKNOWN = non-drivable). Command hysteresis in the controller stops SL/F/SR flicker. **Not done:** remembering obstacles in the camera blind zone (< ~16 cm) — needs odometry. |
| 7 | Robot length used: the four corners of the yawed robot rectangle are checked against the depth row they reach. Unobservable corners (outside FOV) reject the path. Consequence: the "narrow gap" scenario obstacles were narrowed from ±12 to ±11 cm outer edge, because the old geometry was passable only with a width-only check. |
| 8 | Heading command is unchanged (pure-pursuit to the 65 cm lookahead). The steep early slope of the ease-out path is now accounted for by the corner check above. |
| 9 | `nearestObstacleCm` only looks inside the expanded corridor and needs ≥ 2 obstacle pixels on a row. |
| 10 | Ground projection uses true pinhole geometry (atan row mapping, slant-corrected lateral scale). Still needs real calibration; swap `horizontalFovDeg`/`verticalFovDeg` if the phone is mounted in portrait. |

## Motor / signs
| # | Change |
|---|--------|
| 11 | `MIN_MOTOR_PWM = 60` (placeholder — calibrate) in app and sketch; inner wheel of SL/SR floored to it; 120 ms kick-start when starting from standstill at low PWM. |
| 12 | STOP sign: stops once, then ignores the sign for 8 s. NO_ENTRY: centred → stop until it has been out of view 1.5 s; left/right sign → vetoes steering toward that side (**assumption** about sign placement; adjust to your course). |
| 13 | Arduino: no `String` (fixed buffer), speed required for motion commands, `SAFETY_STOP=WATCHDOG` emitted, app parses `RESULT=STOP_*`. `FreeSpaceAnalyzer` scores are still computed but unused by the controller. |

## Tests added
`ObstacleTrackerTest`, `SpeedControllerTest`, and new cases in `NavigationControllerTest` (stale/missing/NO_ECHO ultrasonic, 30 cm stop, stop-sign cooldown, NO_ENTRY, hysteresis) and `ObstaclePlannerTest` (clearance margin, corridor-limited nearest obstacle, noise pixel, projection round-trip).

## Virtual test motion and command-stability patch

- Corrected the synthetic scene timing: the virtual runner samples at 10 FPS and now uses 60 frames (6 seconds) per scenario. The previous 90-frame mask timing did not match the three-second scene cuts in the older MP4.
- The bundled MP4 is now a 48-second clock source. The app renders a matching synthetic first-person POV from the same physical obstacle boxes and pose used to generate the semantic mask; it no longer shows a static pre-recorded view unrelated to the active navigation path.
- The virtual vehicle integrates motion from the stabilized path angle and PWM. Forward position, lateral position, and heading are displayed in the virtual telemetry. Scene resets clear tracking/planner/decision state so stale obstacles do not leak into the next scenario.
- The scene-title text is no longer passed to the traffic-sign model in virtual mode, avoiding false detections caused by text in the old clock video. Live-camera traffic-sign inference is unchanged.
- Added a shared cubic-Bezier trajectory profile with a gentler entry tangent, a reduced wide-gap reward in candidate costs, and continuity cost toward the previously selected lateral offset. The overlay and path drawn on the virtual POV use the same profile.
- Added a virtual-only command stabilizer: geometry STOPs require two consecutive frames, movement resumes after two consistent frames, direction changes require confirmation, and speed ramps gradually. Hard safety reasons (ultrasonic, confirmed sign, stale camera/free-space, unavailable geometry) remain immediate. Live controller behavior is not debounced by this virtual-only filter.
- Added tests for trajectory-profile shape, transient STOP flicker, direction-change confirmation, and immediate hard-stop priority.

The pure Kotlin navigation core and scenario behavior checks passed. A complete Android Gradle build was not run because the environment does not include the required Android SDK/Gradle installation; validate the project in Android Studio before physical operation.
