# v0.5.16 — Speed Profile and Traffic-Sign Verification

## Current speed profile

These are commanded PWM values, not measured road speeds. DC-motor response is nonlinear and depends on battery voltage, load, friction, wheel alignment and the motor driver's losses.

| Behavior | Current setting | Notes |
|---|---:|---|
| Base speed-controller ceiling | 40 PWM | Applied before command-type scaling and obstacle/geometry reductions |
| Straight `F` / `REV` | 80% of base, hard cap 32 PWM | Firmware repeats the 32 PWM cap |
| Normal steering `SL` / `SR` / `L` / `R` | 120% of base, hard cap 48 PWM | A 40 PWM base target becomes 48 PWM before ramping/safety caps |
| Normal slight-turn outer wheel | Minimum 40 PWM | Used with valid sonar so a differential remains after the inner wheel floor |
| Slight-turn outer wheel when sonar is `NO_ECHO` | Maximum 30 PWM | NO_ECHO safety cap takes precedence over the normal 40 PWM outer-wheel minimum; inner wheel floors at 25 PWM |
| Active scan | Start 31; max 41 PWM | The 41 PWM ceiling is used only with a valid sonar echo and adequate turn evidence |
| Active scan with `NO_ECHO` | Maximum 30 PWM | Same cap in Android, Arduino and virtual motor model |
| Motor floor | 25 PWM | Placeholder until tested with the chassis under load |
| Startup kick | 30 PWM for 40 ms | Only when requested PWM is below 28; active scan bypasses it |
| Bounded reverse backoff | 30 PWM for 250 ms | Distance is estimated, not measured; no rear sensor is fitted |

The 20% settings apply to the app's requested PWM target. They do not promise a 20% change in centimetres/second. The motor floor, the normal 40 PWM outer-wheel floor for slight turns, ramp-up, curvature reductions and safety caps may dominate a particular command.

## Traffic-sign class map

The decoder and the packaged `app/src/main/assets/models/traffic_sign_labels.txt` have the same ordered 29-label map. Class IDs start at zero:

| ID | Label | ID | Label | ID | Label |
|---:|---|---:|---|---:|---|
| 0 | `compulsory_left` | 10 | `no_parking` | 20 | `speed_limit_50` |
| 1 | `compulsory_right` | 11 | `no_stopping_or_parking` | 21 | `stop` |
| 2 | `gap_in_median` | 12 | `pedestrian_crossing` | 22 | `turn_left` |
| 3 | `give_way` | 13 | `right_turn_prohibited` | 23 | `turn_right` |
| 4 | `horn_prohibited` | 14 | `right_reverse_bend` | 24 | `u_turn_ahead` |
| 5 | `hump_ahead` | 15 | `roundabout` | 25 | `u_turn_prohibited` |
| 6 | `left_hand_curve` | 16 | `school_ahead` | 26 | `cross_road` |
| 7 | `left_turn_prohibited` | 17 | `side_road_right` | 27 | `parking` |
| 8 | `level_crossing` | 18 | `speed_limit_100` | 28 | `slippery_road` |
| 9 | `no_entry` | 19 | `speed_limit_30` | | |

### What detection means in this release

- Input expected by the packaged model: float32 NCHW `[1,3,640,640]`; output expected: float32 `[1,33,8400]`.
- The decoder considers candidates at confidence >= 0.50 and uses class-aware NMS. The temporal filter requires 3 consecutive matching detections before a candidate becomes the confirmed sign.
- `SIGN CANDIDATE` in LIVE CAMERA telemetry is a raw current model result. `SIGN` is the three-frame confirmed result and is retained through brief misses for up to 1.2 seconds.
- `stop` and `no_entry` are the only classes that currently change navigation behavior. `turn_left`, `turn_right`, `compulsory_left/right`, prohibited turns, speed-limit signs and the other classes are detection/status only; they do not yet tell the robot to choose or execute that turn.
- The virtual obstacle video deliberately does not run sign inference because its synthetic scenes do not have labelled traffic-sign ground truth. `SIGN --` in virtual mode is not a test of the sign model.

### What has and has not been verified

A synthetic-tensor smoke test passed for each of the 29 decoder class IDs, including `turn_left` (22) and `turn_right` (23), and a text check confirmed that the model label asset matches the code list. This verifies output decoding/class-index wiring, **not** that the trained neural network reliably recognizes real signs. Real model accuracy has not been tested against a labelled image set or confirmed on the target phone in this session.

### Manual phone test

1. Open the v0.5.16 project in Android Studio, sync/build it, install it on the phone, and open **LIVE CAMERA**. Do not use the virtual scenario to validate sign detection.
2. Hold one sign at a time in a well-lit view, facing the camera and large enough to be clearly visible; avoid other signs or text in the background for the first test.
3. Watch `SIGN CANDIDATE` for the raw label and confidence. If it stays on the same label through 3 inference updates, check `SIGN` for the confirmed label. A momentary candidate does not mean it is stable enough to be used as behavior.
4. Test `turn_left`, `turn_right`, `compulsory_left`, `compulsory_right`, `left_turn_prohibited`, `right_turn_prohibited`, and each other class separately. Record misses and wrong labels rather than counting only successful detections.
5. Never test by allowing the robot to approach real traffic. The setup is a controlled robotics experiment, not a road-vehicle system.

## Release validation status

- Navigation package compile and speed-profile smoke check: passed.
- Virtual motion source compiled against local Android-graphics stubs; virtual NO_ECHO turn test confirmed lower yaw than a valid-sonar turn at the same requested PWM.
- Arduino sketch syntax check and speed-cap runtime smoke test with Arduino API stubs: passed.
- Decoder mapping smoke test: all 29 labels passed.
- Full Android Gradle build was not completed in this environment (the required Gradle 9.6+ distribution / Android SDK are not available here).
- Physical movement and real-camera model accuracy still require testing on the target chassis and phone.
