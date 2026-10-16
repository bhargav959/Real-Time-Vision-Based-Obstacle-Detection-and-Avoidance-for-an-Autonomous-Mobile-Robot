# Simulation/Physical Parity and Calibration — v0.5.10

## Audit findings fixed in this release

1. **Gap policy mismatch:** planner instances now share the app's 2 cm per-side clearance margin. A 15 cm chassis therefore requires a 19 cm clear gap in both virtual and live planning. The 10 cm preferred-boundary margin remains a separate policy.
2. **Idealized virtual movement:** the old simulator followed the planner's ideal selected curve. The virtual robot now responds to the same discrete `F`, `SL`, `SR`, `L`, `R`, `REV`, `BACKOFF`, and `S` outputs using a differential-drive approximation, PWM floor, slight-turn wheel ratio, startup-kick timing and swept-footprint collision checking.
3. **Missing virtual ultrasonic behavior:** virtual mode models a forward-mounted HC-SR04-like beam against the synthetic obstacle map, reports `NO_ECHO` when no obstacle is hit, and uses the same 30 cm translation stop, 24 cm pivot gate, 22 cm camera clearance gate, and 90 PWM no-echo cap as live mode. The simulated beam is deterministic and still cannot reproduce acoustic dropouts/material effects.
4. **Command timing inconsistency:** v0.5.9 used a normal held `L`/`R` command and relied on Android issuing STOP around the next 100 ms tick. Scheduler delays could extend a physical pulse while virtual mode always rotated exactly one frame. v0.5.10 adds `SCAN_L`/`SCAN_R` firmware keepalive commands; the Arduino maintains the turn only while Android refreshes the same direction and stops after a 250 ms lease timeout. Four left pulses and up to eight right pulses remain bounded.
5. **Low-speed steering mismatch:** the virtual model now copies the Arduino's inner-wheel PWM floor (60), slight-turn ratio (45%), outer-wheel minimum (80), and startup kick (PWM 90 for 60 ms only when the requested top wheel PWM is below 80). The firmware already applies the same outer-wheel minimum so `SL/SR` do not collapse to straight motion at low requested PWM.
6. **Reverse-distance mismatch:** v0.5.9 advanced the virtual robot by a forced 5 cm while hardware only received a timed 250 ms, PWM-80 command. v0.5.10 advances by the distance predicted from the same PWM/time model instead. With the default placeholder of 18 cm/s at PWM 150, the current model predicts 9.6 cm/s at PWM 80 and 2.4 cm in 250 ms—not 5 cm. The placeholder must be calibrated to the actual robot; this is intentionally visible rather than hidden.
7. **Heading saturation:** virtual yaw is now normalized for display rather than artificially clamped at ±65 degrees while physical pivot commands could continue turning.

## What is shared, and what is still different

| Behavior | Live hardware | Virtual mode in v0.5.10 |
|---|---|---|
| Gap requirement | 19 cm effective | 19 cm effective |
| Command stabilization | Same class: immediate STOP, two-tick direction confirmation, PWM ramp | Same class |
| Active scan | 100 ms same-direction keepalive pivot steps with no in-sweep motor stop; 250 ms firmware lease and stable-view stops after each sweep | Same state machine, modeled 100 ms commanded scan step |
| Backoff | PWM 80 for 250 ms, rear not sensed | Uses same PWM/time model and swept rear collision guard |
| Front sonar | Real HC-SR04, synchronous safety pings plus 400 ms telemetry polls | Synthetic rays against known world geometry, 100 ms updates |
| Obstacles | Neural-model predictions + approximate ground projection | Synthetic ground-truth mask + known obstacle world |
| Pose feedback | No encoders/IMU; actual movement is unmeasured | Mathematical differential-drive estimate |
| Motor variability | Battery, gearboxes, mismatch, friction, wheel slip | Single nominal speed calibration unless configured otherwise |

The virtual harness is now a more representative control simulation, but it remains optimistic about perception because it uses synthetic ground truth rather than actual camera-model errors.

## Required physical calibration

Perform calibration in a wide, enclosed, obstacle-free area, with the emergency STOP within reach. Do not enable autonomous motion near people, pets, stairs, traffic, or fragile items.

1. **Wheel direction:** lift wheels off the floor. Briefly test `F`, `REV`, `L`, `R`, `SL`, and `SR`. Confirm both motor directions match the UI. Fix `INVERT_LEFT_MOTOR` / `INVERT_RIGHT_MOTOR` or wiring first if not.
2. **Loaded PWM-speed curve:** mark the floor over a known 50–100 cm route. Run straight at several fixed PWM values (at least 60, 80, 100, 120), repeat each trial three times, measure distance/time after startup transients, and calculate loaded cm/s. Update `nominalWheelSpeedCmPerSecAtPwm150` in the `VirtualScenarioProvider` constructor/default: `measured speed at PWM p × 150 / p` is a first-order estimate. If left/right wheel speeds differ, the single-speed model remains approximate and should be extended with per-wheel gains.
3. **Wheel track:** measure the distance between the left/right wheel contact-centre lines; replace `WHEEL_TRACK_WIDTH_CM` (default 13.5 cm) in `MotorCalibration`. Use small pivot commands to validate that predicted yaw is close to camera/angle measurements. Ground friction may still vary pivot rate.
4. **Bounded scan pivots:** confirm `SCAN_L` and `SCAN_R` rotate in the expected direction at low speed; confirm the next keepalive maintains the turn and that absence of keepalives stops within 250 ms. A manual STOP must halt immediately.
5. **Minimum moving PWM and slight turns:** verify PWM 60 moves both wheels under load. Check `SL/SR` at PWM 80, because the firmware enforces an outer wheel PWM of at least 80 and an inner wheel floor of 60. Increase the outer/inner ratio only after controlled measurements; do not use a very low value that makes both wheels equal after flooring.
6. **Backoff distance:** mark a starting line and run one PWM-80, 250 ms `BACKOFF` in an open space with the rear clear. Repeat three times. If mean distance is `d` cm, the measured equivalent speed at PWM 150 is approximately `d / 0.25 * 150 / 80` cm/s. Update the shared virtual speed calibration or firmware pulse duration so the virtual prediction matches the measured chassis. Never assume 250 ms means 5 cm by default.
7. **Ultrasonic range:** compare real readings with a ruler at 20, 24, 30, 40 and 60 cm. Test angled, soft, narrow and off-centre obstacles. The virtual beam cannot model all acoustic misses; retain conservative safety margins even after calibration.
8. **Camera geometry:** verify actual phone height, downward tilt and FOV; compare detected distance to ruler measurements at multiple depths. Recalibrate the 10 cm/7.5° defaults if necessary.
9. **Speed/stop distance:** measure stopping distance from PWM 80 and 120. Maintain a buffer that includes sensor age, the 100 ms control tick, motor coast and any ultrasonic blocking time. Do not reduce the 30 cm app translation threshold without measured evidence.

## Regression tests performed / limitations

The source includes tests for differential-drive turning, simulated sonar front offset, shared 19 cm clearance, timed-PWM backoff prediction, and active scan state timing. Scan pivot commands are explicitly represented in the Android serial allowlist and matching Arduino parser. The full Android Gradle build requires Gradle 9.6+ and an Android SDK; the current build environment does not provide that toolchain. No physical robot was available for acceptance tests. A release should be considered hardware-validated only after the above checklist has been run on the actual chassis.
