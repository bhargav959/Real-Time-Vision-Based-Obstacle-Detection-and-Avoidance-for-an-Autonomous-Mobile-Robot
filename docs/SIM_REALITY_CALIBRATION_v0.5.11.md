# Sim-to-Real Hardening and Calibration — v0.5.11

> **Current-release note (v0.5.21):** this document records historical settings. Current firmware rate-limits HC-SR04 triggers to at least 60 ms apart and takes one reading per gate. One short blind `BACKOFF` pulse is enabled, but continuous `REV` remains disabled. A fresh post-reverse map and two-sided scan are required before a route can resume. The rear space remains unsensed and must be physically clear before testing.

v0.5.10 made the *commands* match between the virtual test and the robot. v0.5.11 closes the remaining gap: the
virtual world was too clean (zero latency, perfect masks, ideal motors, fresh sonar every frame). This release
(a) fixes real-world timing/safety weaknesses in the live path and (b) makes the virtual test inject the
imperfections the real robot has, so a pass means more.

No physical robot was available. Everything below is verified by 72 JVM unit/closed-loop tests (see "Verification"),
not by hardware. **Treat the release as hardware-validated only after the checklist at the end.**

## What changed

### Live path (real robot)
| # | Change | Why |
|---|---|---|
| 1 | Firmware appends the filtered sonar reading to every motion ack: `RESULT=F,90,D=41.2`. App parses it (`SerialProtocol.parseUltrasonic`). Idle `DISTANCE` poll 400 → 150 ms. `ULTRASONIC_MAX_AGE_MS` 800 → 400. | Live sonar was up to 400–800 ms old (≈6–11 cm of blind travel at PWM 120) while the sim gave a fresh reading every 100 ms. |
| 2 | Firmware in-motion guard: while F/SL/SR/L/R is active it pings every 60 ms and stops on two consecutive readings below the limit (one reading below 70 % of it). | Previously the sonar was only read when a command arrived; an app stall left ~400 ms of driving. |
| 3 | `getDistanceFilteredCm` inter-ping gap 3 → 10 ms (and no trailing delay). | 3 ms let a previous echo contaminate the next ping. |
| 4 | Free-space map age is measured from **frame capture** (`CameraAnalyzer` timestamps the frame), not from the end of inference. `SCAN_EVIDENCE_MAX_AGE_MS = 500`. | Inference + tracker latency (300–500 ms) was invisible to every freshness check. |
| 5 | Scan sampling requires a frame captured **≥100 ms after the STOP**, and (with gyro) a yaw rate under 4 °/s. 900 ms timeout → abort. | The old 200 ms settle could sample a pre-stop, motion-blurred frame. |
| 6 | **Gyro-verified scan pivots** (`YawTracker` + `YawSensor`, phone gyro, mounting-independent). A pulse that rotates < 1° is "ineffective": PWM rises 80 → 90 → 100; still no rotation twice → scan aborts. No gyro = old open-loop behaviour. | Open-loop 80 ms pulses may not move the chassis at all (static friction/low battery); the scan then "sampled" an unchanged view. |
| 7 | `LiveDecisionStabilizer`: F ↔ SL ↔ SR changes no longer stop the robot. Pivots/reversals still stop + confirm; every safety STOP is still immediate. | One flickering segmentation frame cost 200 ms of stop and a motor re-kick; the noise-free sim never showed it. |
| 8 | New config `cameraForwardOffsetCm`, `sonarForwardOffsetCm` (in `MainActivity.geometryConfig`). The planner measures distances from the pivot centre. | The camera was assumed to sit over the pivot centre; a nose-mounted phone shifts every distance by ~10 cm. |
| 9 | Shadow mode: in SAFE the app logs `SHADOW` decisions (what it *would* send). | Safe way to test perception on the real floor by hand-pushing the robot. |

### Virtual test (`SimRealism`, on by default in the app: `SimRealism.REALISTIC`)
Perception latency (3 frames = 300 ms), per-frame missed-obstacle patches and false obstacle blobs, ±1 px mask
jitter, right-wheel gain 0.93 (drift), motor dead-band (45 PWM), first-order motor spin-up (τ = 60 ms),
25 % pivot scrub loss, sonar dropouts (4 %) / spurious near echoes (1 %) / σ 0.8 cm noise. Seeded, so repeatable.
`SimRealism.IDEAL` restores the old behaviour. **These numbers are placeholders; replace them with measurements.**

> **Version note:** Sections above the v0.5.14 appendix record the settings and behavior of older releases. For current v0.5.14 scan PWM and keepalive timing, the appendix at the end of this file is authoritative.

## Verification performed (no hardware)
* Historical v0.5.11 verification: 72 JUnit tests passed at that version. The v0.5.14 appendix below lists checks rerun after the keepalive scan change.
* `ClosedLoopSimTest`: headless copy of the app's virtual pipeline, 9 scenes × 25 random seeds under REALISTIC:
  **0 / 225 collisions.** Sensitivity (not asserted): 0/90 collisions at 2× chassis speed with 5 frames of latency;
  1/90 at 2× speed + 15 % sonar dropouts — i.e. there is margin, but it is finite.
* Arduino sketch compiles clean (`g++ -Wall -Wextra`) against a mock Arduino header. It has **not** been run on a UNO.
* **Not compiled here:** `MainActivity.kt`, `CameraAnalyzer.kt` (need the Android SDK). They were reviewed line by line;
  `YawSensor.kt` was compiled against minimal Android stubs. Build in Android Studio and read any error first.

## Updated physical calibration checklist
Do the v0.5.10 checklist first (wheel direction, speed curve, track, scan pivots, backoff, sonar, camera, stop distance),
with these amendments, in a wide enclosed area with the STOP button within reach:

1. **Wheel speed is not linear through zero.** Measure loaded cm/s at PWM 35, 40, 45, 50, 55 and 60. Fit
   `v = k·(pwm − pwm0)`; set `motorDeadbandPwm = pwm0` and `NOMINAL_SPEED_CM_S_AT_PWM_150` in `SimRealism`/`MotorCalibration`.
2. **Effective track width from rotation, not a ruler.** Run a low-speed `SCAN_L,50` keepalive every 100 ms for 2 s in a clear enclosure and measure degrees turned (the app's
   gyro yaw is logged); `track_eff = 2·v / ω`. Put it in `WHEEL_TRACK_WIDTH_CM`. Set `pivotSlipLoss` from the gap vs geometry.
3. **Scan pulse effectiveness.** Run `SCAN_L,35` every 100 ms for four scan steps; record yaw per step. If the phone gyro observes less than 0.5° or rotation is inconsistent, the controller may raise the next step by 5 PWM up to the current cap (60 with valid sonar; 45 on NO_ECHO). Do not raise the cap without measurement.
4. **Motor mismatch.** Drive L = 100 cm straight at PWM 50 and measure the heading change θ (rad; the logged gyro yaw works). Then `rightWheelGain ≈ 1 − θ·track_eff / L` (sign: drift toward the right = right wheel slower). Consider per-wheel PWM trim in firmware.
5. **Battery.** Repeat 1–3 at full and ~50 % charge, staying within PWM 35–60. If speeds differ > 20 %, calibrate at the lower value or add a voltage-divider on A0 and compensate.
6. **Camera/sonar offsets.** Measure from the pivot centre (midpoint between wheels) to the camera lens and to the sonar
   face; set `cameraForwardOffsetCm` and `sonarForwardOffsetCm`. Re-check the 22/24/28/30 cm gates afterwards.
7. **Projection test.** Place boxes at 30/40/60 cm and ±10 cm laterally; compare `nearestObstacleCm` and gap widths in the
   telemetry with a ruler. Include two *tall* objects with a 19 cm gap: tall objects project to oversized ground footprints
   (camera is only 10 cm high), so live may reject gaps the flat-box sim accepts. If so, widen the real gap rules or tune the
   camera tilt/FOV before trusting passes.
8. **Perception latency.** Log `freeSpaceAgeMs` at decision time while driving; set `perceptionLatencyFrames` in `SimRealism` to match.
9. **Shadow-mode drive.** In SAFE mode, push the robot by hand through the real obstacle course and read the `SHADOW` lines in the
   log. Every decision should match what a person would do. Fix perception before enabling motors.
10. **Electrical.** 10 kΩ pull-downs on ENA/ENB (and IN1–IN4); 100 nF across each motor; motors on their own battery with common GND.
    Watch for `ROBOT_READY` appearing mid-run (= brown-out reset).
11. **Stopping distance** with the new firmware guard at PWM 50 and 60; measure it on the actual chassis before raising any limit.

## Known remaining differences (not fixable in software alone)
* Ultrasonic and camera still miss thin/soft/angled objects inside the camera's ~16–20 cm blind zone. Add side/forward
  sensors or a bumper switch if the robot will meet chair legs or table edges.
* No cliff/stair detection. Keep it away from drops.
* The reverse (`BACKOFF`) pulse is still unsensed at the rear.
* The sim obstacles are flat boxes; real tall obstacles occlude the floor behind them.


## v0.5.14 low-speed active scan update

- App and Arduino hard PWM cap are synchronized at 60. The normal startup kick is 50 PWM for 60 ms, but active scan pivots bypass it.
- Active scan starts at 35 PWM and can increase in 5-PWM increments only to 60 PWM. When phone gyro feedback is unavailable, each speed is used for two pulses before increasing. If the chassis cannot pivot below the cap, calibrate the real motor deadband rather than silently raising the cap.
- Up to four left scan steps run continuously while the app refreshes the same-direction command every 100 ms. The Arduino stops the scan if a refresh is missing for 250 ms. The robot stops after the left sweep to acquire a fresh stable view, then runs up to eight right steps and stops after that sweep. Any safety gate failure still stops immediately.
- This version does not eliminate all stopping: it removes the stop/sample delay after every individual pulse. The pauses at the end of each side sweep and all emergency/safety stops are intentional.


## v0.5.14 validation performed

- Core navigation state-machine Kotlin sources compiled with `kotlinc`; a targeted smoke test passed for backoff → consecutive low-speed scan steps → stable-view stop → reverse-direction sweep.
- The Arduino sketch passed a `g++ -std=c++11 -Wall -Wextra -fsyntax-only` check against Arduino API stubs. A simulated firmware harness passed: the pivot stays powered across same-direction keepalives, updates PWM without a stop, stops when the 250 ms lease expires, and stops on a close sonar reading. A separate Kotlin smoke check confirmed the active scan never commands above 45 PWM when the required ultrasonic reading is NO_ECHO.
- Full Android Gradle tests and device build were not run here because the required Gradle 9.6+ / Android SDK toolchain is unavailable. No physical robot test is claimed.
