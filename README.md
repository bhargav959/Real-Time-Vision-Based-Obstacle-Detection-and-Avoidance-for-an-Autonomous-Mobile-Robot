# RobotCarAI Android — Obstacle Navigation v0.5.21

> **v0.5.21 (bounded reverse + verified avoidance):** one reverse pulse is enabled at 25 PWM for at most 250 ms; continuous `REV` remains disabled. After backoff, the app stops, requires a camera/free-space sample captured after reverse ends, scans both sides, and resumes only when a fresh path is verified. If an obstacle still blocks the centre corridor, a straight `F` command is rejected and the selected left/right avoidance direction must match the planner's signed path. The rear is not sensed: reverse is still blind and must only be tested in a supervised area known to be clear behind the robot.

> **v0.5.17 traffic-model update:** the supplied `best.pt.zip` checkpoint is exported to float32 TFLite at 640x640. The app decoder and label asset follow the checkpoint class order; IDs are reported in `docs/TFLITE_MODEL_REPORT_v0.5.17.json`. The free-space ONNX model and model labels remain unchanged.

> **v0.5.16 (turn/straight profile + traffic-sign diagnostics):** the base cruise controller stays at 40 PWM; the app applies 80% to F/REV targets (max 32 PWM) and 120% to normal steering targets (max 48 PWM). Active scan starts at 31 PWM and may rise to 41 with valid sonar; NO_ECHO remains capped at 30 PWM. Firmware enforces a 48 PWM absolute ceiling, 32 PWM F/REV ceiling, and 40 PWM outer-wheel floor for SL/SR when sonar is valid (clamped to 30 on NO_ECHO so the safety cap wins). The motor floor (25 PWM) and wheel response remain chassis-specific and need calibration under load. Traffic-sign telemetry now shows raw candidates separately from 3-frame confirmed signs; confirmation expires after 1.2 s. Only STOP and NO_ENTRY have direct behavior; other sign classes are status-only.

> **v0.5.13 (robust left/right avoidance):** the planner evaluates left and right routes independently, retains its chosen avoidance side through brief segmentation flicker, and releases the side lock only after the centre path is verified clear and the trajectory has returned near centre. A blocked centreline can no longer be treated as FORWARD just because its avoidance heading is under 5°; the controller emits SL/SR, and requests L/R pivots inside the front-stop zone only when the existing independent camera-clearance and fresh ultrasonic gates pass. Added focused tests for small-angle direction selection and close-range pivot safety. Full Android build still needs to be run in Android Studio.

> **v0.5.11 (sim-to-real hardening):** fresher sonar (firmware piggybacks the reading on every ack + autonomous in-motion guard), camera-capture-based data ages, gyro-verified scan pivots, no stop on steering flicker, camera/sonar mounting offsets, shadow-mode logging, and a virtual test that now injects latency, perception errors, motor dead-band/lag/mismatch and sonar dropouts (`SimRealism`). See `docs/SIM_REALITY_CALIBRATION_v0.5.11.md`. Firmware and app must be updated together (the firmware acks now carry `,D=`).

# (previous) v0.5.10

> v0.5.10 audits live-vs-virtual behavior and makes the simulation follow discrete wheel/PWM commands rather than an ideal planner curve. Live and virtual planning share the 19 cm effective gap rule (15 cm body plus 2 cm per-side margin), command stabilizer, 100 ms scan-command refresh with a 250 ms firmware lease, and modeled ultrasonic gate. The 250 ms `BACKOFF,50` pulse is simulated from the configured speed model; its physical distance is not guaranteed to be 5 cm until the chassis is calibrated. Rear clearance remains unsensed and must be checked by the operator.

This release removes lane/black-line following and replaces it with local obstacle-aware navigation.

## Navigation behavior

The phone camera feeds the supplied semantic free-space model. The app projects the segmentation onto an approximate ground plane, estimates obstacle geometry and passable gaps, applies a vehicle-centered virtual boundary, selects a path, and assigns a speed.

```text
CameraX
  -> free-space segmentation
  -> obstacle / gap geometry
  -> virtual boundary policy
  -> straight-path preference or avoidance curve
  -> speed controller
  -> COMMAND,SPEED over USB
  -> Arduino
  -> L298N + motors
```

Traffic-sign detection remains active. Confirmed `STOP` and `NO_ENTRY` events retain priority.

## Physical configuration

- Robot width: **15.0 cm**
- Robot length: **25.5 cm**
- Nominal boundary: **10 cm beyond each robot side**
- Temporary avoidance extension: **20 cm beyond the nominal boundary**
- Camera height: **10 cm above ground**
- Camera downward tilt: **~7.5° configured value**
- Per-side gap safety margin used by the app: **2.0 cm**
- Effective minimum gap for both live and virtual planning: **19.0 cm**

The nominal boundary is a preferred operating corridor. The planner first tries to find a feasible path inside it. Only when no feasible 19 cm effective gap exists there does it allow a temporary boundary crossing inside the expanded region. The 10 cm nominal boundary margin is not added to the gap-fit rule; the separate 2 cm per-side clearance is.

## Path selection

Two path types are supported automatically:

1. **STRAIGHT** — used whenever the center/nominal path is feasible or the road is clear.
2. **AVOIDANCE_CURVE** — generated from the lateral offset to a selected safe corridor when an obstacle blocks the preferred route.

The planner evaluates left/right corridors independently, prefers the wider verified gap when the choice is new, and retains its chosen side through brief perception changes. The side lock is released only after the centre corridor is repeatedly verified clear and the trajectory has returned near centre. If the centre corridor is blocked, the controller sends `SL` or `SR` even for small steering angles; it never rounds a necessary avoidance curve to `F`. When the front sonar enters the 30 cm stop zone, the controller requests an in-place `L`/`R` pivot only if sonar is at least 24 cm, camera geometry reports at least 22 cm nearest-obstacle clearance, and the selected corridor is at least 3 cm wider than the configured fit width. Otherwise it sends `S`. If no verified corridor exists, the car stops and the bounded active-scan process may run only under its existing fresh-evidence safety gates.

## Gap evaluation

For each forward planning band, the app evaluates the obstacle mask across the robot's approximately 25.5 cm longitudinal footprint. The app adds a 2 cm margin to each lateral side of the 15 cm body, so both live and virtual planner instances require 19 cm of measured free gap.

```text
usable_gap >= 19 cm  -> PASSABLE
usable_gap < 19 cm   -> REJECT
```

The 10 cm preferred boundary margin is not added to this test; the separate 2 cm per-side clearance is.

## Speed control

The phone sends a PWM speed with each movement command. In v0.5.16, the base speed controller targets at most 40 PWM, F/REV are scaled to 80% and capped at 32 PWM, steering commands are scaled to 120% and capped at 48 PWM, and the slight-turn outer wheel minimum is 40 PWM with valid sonar (clamped to 30 on NO_ECHO). Active scan starts at 31 PWM and may rise by 5 to 41 PWM with valid sonar, or 30 PWM on NO_ECHO. In v0.5.21, blind backoff is capped at 25 PWM for 250 ms; the required post-reverse fresh-map scan prevents immediate resumption on a pre-reverse path. The 30 PWM startup kick lasts 40 ms and only applies below a requested PWM of 28; active scan bypasses it. The controller reduces speed for tighter curves, boundary crossing, narrow passages, and close obstacles. These are conservative starting values, not measured motor speeds; low PWM may stall a loaded chassis, so calibrate the minimum reliable wheel speed.

## USB protocol

Android -> Arduino:

```text
F,32       forward (straight-motion cap)
SL,48      slight left (may be capped at 30 on NO_ECHO)
L,48       hard left (may be capped at 30 on NO_ECHO)
SR,48      slight right (may be capped at 30 on NO_ECHO)
R,48       hard right (may be capped at 30 on NO_ECHO)
REV,32     reverse (manual; rear is not sensed)
BACKOFF,25 one-shot blind reverse pulse, firmware-limited to 250 ms
SCAN_L,31 low-speed left scan pivot held by 100 ms keepalive; 250 ms lease
SCAN_R,31 low-speed right scan pivot held by 100 ms keepalive; 250 ms lease
S,0        stop
DISTANCE   request ultrasonic distance
```

Every line is newline terminated at 115200 baud, 8-N-1.

Arduino -> Android telemetry can include:

```text
DISTANCE=42.5
RESULT=SR,48
SAFETY_STOP=DISTANCE:22.0
```

Matching UNO firmware is in `docs/arduino/RobotCarObstacleNavigation.ino`. The timed backoff is only an estimate without wheel encoders; keep the area behind the robot clear before testing because the HC-SR04 faces forward.

## Arduino wiring for this release

```text
L298N ENA -> Arduino D9  (PWM, jumper removed)
L298N IN1 -> Arduino D2
L298N IN2 -> Arduino D3
L298N IN3 -> Arduino D4
L298N IN4 -> Arduino D5
L298N ENB -> Arduino D10 (PWM, jumper removed)

HC-SR04 TRIG -> Arduino D6
HC-SR04 ECHO -> Arduino D7
```

Use a common ground between the Arduino, L298N logic side, and HC-SR04.

## Virtual test

The bundled `robot_virtual_test.mp4` is a 54-second timing clock for nine deterministic obstacle-navigation scenarios. The app renders a moving first-person POV from the same world-space geometry used for planning. Scenarios include open travel, center blockage, narrow/exact-width gaps, left/right avoidance, multiple obstacles, a full-view wall that triggers active path discovery, and a dead end.

The virtual harness derives a deterministic synthetic obstacle mask and ultrasonic readings from the same known world map. It deliberately bypasses AI recognition so planner/control scenarios are reproducible; this is still more optimistic than live camera perception. The simulated vehicle now moves from discrete `F/SL/SR/L/R/REV` motor commands using a differential-drive/PWM model, startup-kick behavior and collision checks rather than snapping to an ideal selected curve.

The overlay shows:

- white: nominal physical boundary;
- amber: temporary expanded boundary;
- green: feasible/authorized selected path;
- amber dashed: scan direction not yet verified.

The telemetry panel reports command, PWM, path type, path angle, boundary state, obstacle estimate, gap width, ultrasonic reading, and reason.

## Traffic-sign classes and verification

The v0.5.17 decoder labels are synchronized to the uploaded checkpoint. `turn_left` is class not present in checkpoint, `turn_right` is class 6, `stop` is class 5, and `no_entry` is class not present in checkpoint. Live telemetry separates `SIGN CANDIDATE` (raw result) from the `SIGN` (same class confirmed for three consecutive inference updates). Only `stop` and `no_entry` currently affect motion; other signs are displayed as detections and do not automatically steer the robot. The virtual obstacle video does not run sign inference. Decoder index checks passed for all 29 labels, but learned model accuracy still needs a real-image / on-phone test. See `docs/TRAFFIC_SIGN_AND_SPEED_PROFILE_v0.5.17.md` for the full class map and test procedure.

## Camera metric calibration

The app uses a configurable pinhole ground-plane approximation with 62° horizontal FOV, 48° vertical FOV, and a 75 cm local lookahead. The supplied 10 cm mounting height and approximately 7.5° downward angle are used as the initial configuration.

Exact centimetre accuracy still requires a one-time calibration against a known floor distance with the actual phone. The geometry constants are intentionally centralized so that calibration does not require changing the planner logic.

## Project structure

```text
app/src/main/java/com/example/robotcar/
  MainActivity.kt
  ai/
    ModelRegistry.kt
    OnnxModelRunner.kt
    TfliteModelRunner.kt
    TrafficSignDecoder.kt
  camera/
    CameraAnalyzer.kt
  navigation/
    Models.kt
    ActivePathDiscovery.kt
    NavigationGeometry.kt
    ObstaclePlanner.kt
    SpeedController.kt
    NavigationController.kt
    NavigationOverlayView.kt
    FreeSpaceAnalyzer.kt
    TrafficSignFilter.kt
  communication/
    UsbSerialManager.kt
  video/
    VideoTestRunner.kt
    VirtualScenarioProvider.kt
  vision/
    ModelInputPreprocessor.kt
```

The old lane/line-following implementation is no longer part of the navigation package.

## Build note

Open the project in Android Studio and sync with JDK 17. A full Android Gradle build was not available in the creation environment because Gradle 9.6+ and the Android SDK were not installed there.

For current implementation details and validation notes see `docs/ARCHITECTURE.md`, `docs/IMPLEMENTATION_STATUS.md`, and `docs/SIM_REALITY_CALIBRATION_v0.5.10.md`. `docs/OBSTACLE_NAVIGATION_v0.5.1.md` is historical.

## Virtual-test history

The early v0.5.3 simulator followed an ideal latched curve, which made virtual motion more predictable than a real open-loop chassis. That behavior is retained in the historical refinement notes but is superseded by v0.5.10, where the virtual pose is driven by discrete motor commands and an approximate differential-drive model. Virtual testing still uses synthetic ground-truth perception, not a live camera/segmentation-model loop.

See `docs/REFINEMENT_NOTES_v0.5.3.md` for the historical changes and `docs/SIM_REALITY_CALIBRATION_v0.5.10.md` for the current model.

## Active path discovery (v0.5.4)

When an explicit obstacle leaves no verified candidate path for three consecutive frames, the app can initiate a bounded scan. It performs a bounded sequence of low-PWM left scan steps, with the Arduino maintaining the pivot while a same-direction command is refreshed each control tick. A 250 ms dead-man lease stops motion if the app or USB stalls. It stops once per side to allow a fresh camera sample; there is no motor stop between individual same-direction steps, but any safety gate failure stops immediately. If a route becomes feasible, control returns to the normal planner; if not, the robot remains stopped. Traffic-sign, stale-data, USB, and user stops remain hard stops. The only close-ultrasonic exception is the one-shot reverse pulse for a fresh, vision-confirmed blocked path; turning remains disabled until the normal clearance checks pass.

Live scanning requires a ready Arduino, fresh camera/free-space results, an estimated pivot-clearance check, and a fresh ultrasonic reading. The virtual harness uses a swept-circle collision check and keeps physical motor output disabled. See `docs/ACTIVE_PATH_DISCOVERY.md`.


## Bounded path-scan backoff (v0.5.8)

Historical v0.5.8 behavior used `BACKOFF,50`. In v0.5.21, the Android controller sends only a `BACKOFF` pulse capped by firmware at 25 PWM for 250 ms, then stops and requires a post-reverse camera/free-space sample and a left/right scan. It never resumes straight `F` while the centre corridor is blocked; when a safe avoidance route cannot be proven, it stays stopped. The physical distance depends on loaded motor speed, battery and floor. **The physical robot has no rear-facing sensor, so verify the space behind it is clear before testing.**

## Communication reliability update (v0.5.5)
See `docs/COMMUNICATION_RELIABILITY_v0.5.5.md` for USB serialization, fault recovery, Arduino safety interlock, polling and PWM startup updates.


## Live physical-vs-virtual behavior (v0.5.10)

Live and virtual modes now share the same planner clearance margin, temporal obstacle tracker policy, live command stabilizer, scan state machine, PWM floor, slight-turn wheel ratio, startup-kick thresholds, and front-sonar stop/pivot thresholds. Virtual mode simulates the HC-SR04 from the known obstacle map and moves the chassis from motor commands with differential-drive kinematics; it no longer assumes the selected path is followed perfectly. Remaining differences are deliberate: virtual perception uses synthetic ground truth, while live perception uses a fallible neural model and metric camera projection; the physical chassis has wheel slip, motor mismatch, battery variation and no encoders/IMU. See `docs/SIM_REALITY_CALIBRATION_v0.5.10.md` for the calibration sequence.

Before physical use: lift the wheels and verify forward/reverse and left/right polarity; put the chassis on the floor and calibrate minimum moving PWM and turn pulse duration; verify camera height/tilt/FOV; test first at low speed in a clear, bounded space. The rear has no sensor, so the one-shot reverse must only be tested when the area behind the robot is known clear.


## Simulation/physical parity audit (v0.5.10)

- Fixed the gap-rule contradiction: the app's 2 cm per-side safety margin means 19 cm is required, not 15 cm. Both planner instances now use that same config.
- Replaced ideal trajectory-following in virtual mode with discrete motor-command kinematics, including Arduino PWM floor, slight-steering ratio, low-speed startup kick, swept-footprint collision checks and a simulated forward ultrasonic beam.
- Reused the same command stabilizer and active path discovery state machine in both modes; scan pivots use `SCAN_L`/`SCAN_R` keepalive commands starting at PWM 35; the Arduino holds the turn while same-direction commands refresh each 100 ms and stops after a 250 ms command-lease timeout. The app stops after each complete sweep to sample a stable image.
- Removed the hard virtual heading clamp so simulated yaw does not stop at an artificial ±65° while physical motors would continue to turn.
- Backoff distance is now computed from the modeled PWM and 250 ms firmware pulse instead of forcing the pose to move 5 cm. The default speed estimate is an uncalibrated placeholder; see the calibration guide.

The Android Gradle build and physical chassis tests were not completed in the build environment. Treat the virtual model as a better approximation, not a certificate of safe real-world navigation.


## Custom traffic-sign model update (v0.5.12)

The packaged `app/src/main/assets/models/traffic_sign.tflite` is the custom 7-class Ultralytics model converted from `best.pt` (classes: compulsory_right, no_stopping_or_parking, right_turn_prohibited, right_reverse_bend, side_road_right, stop, turn_right). The Android decoder and model runner are configured for a float32 640x640 NCHW input and an `[1,11,8400]` output tensor (4 normalised box values + 7 class scores). Label order is documented in `app/src/main/assets/models/traffic_sign_labels.txt` and matches the metadata embedded in the model. Only `stop` changes robot behavior; the model has no `no_entry` class, so the NO_ENTRY veto cannot trigger (a warning is logged at start-up). Validate on held-out images and test with the drive wheels raised before allowing any physical motion.
